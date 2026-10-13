/*
 * ProMicroDualUART - dual USB CDC <-> UART bridge for the ATmega32U4.
 *
 * This file is the USB side: it moves data between the CDC endpoints and
 * each port's rings, applies line coding / break requests at safe points
 * (after the transmitter has drained), and reports line errors back to the
 * host as CDC SERIAL_STATE notifications.
 */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <LUFA/Drivers/USB/USB.h>

#include "config.h"
#include "Descriptors.h"
#include "clock.h"
#include "hwuart.h"
#include "softuart.h"
#include "gpio.h"

#define NUM_PORTS 2

/* SERIAL_STATE bits that report events rather than a level; the CDC spec
 * says they are cleared once sent. */
#define SERIAL_STATE_EVENTS  (CDC_CONTROL_LINE_IN_BREAK | CDC_CONTROL_LINE_IN_RING |       \
                              CDC_CONTROL_LINE_IN_FRAMEERROR | CDC_CONTROL_LINE_IN_PARITYERROR | \
                              CDC_CONTROL_LINE_IN_OVERRUNERROR)

#define PEND_CONFIG     _BV(0)
#define PEND_BREAK_ON   _BV(1)
#define PEND_BREAK_OFF  _BV(2)

#define LED_PULSE       CLOCK_MS(20)

typedef struct
{
	const uart_port_t*  uart;
	CDC_LineEncoding_t  line;              /* last line coding requested by the host */
	uint8_t             pending;           /* PEND_* */
	bool                break_active;
	bool                break_timed;
	uint16_t            break_ms;
	uint32_t            break_end;
	bool                in_last_full;      /* last IN packet was full size: a ZLP may be due */
	uint16_t            serial_state;
	uint16_t            serial_state_sent;
	bool                rx_full;           /* RTS held off for lack of buffer space */
	bool                cts_armed;         /* FLOW_AUTO: CTS seen asserted since open */
	bool                ri_last;
} bridge_t;

static USB_ClassInfo_CDC_Device_t cdc[NUM_PORTS] =
{
	{
		.Config =
			{
				.ControlInterfaceNumber = INTERFACE_ID_CDC0_CCI,
				.DataINEndpoint         = {.Address = CDC0_TX_EPADDR, .Size = CDC_TXRX_EPSIZE, .Banks = CDC_TXRX_BANKS},
				.DataOUTEndpoint        = {.Address = CDC0_RX_EPADDR, .Size = CDC_TXRX_EPSIZE, .Banks = CDC_TXRX_BANKS},
				.NotificationEndpoint   = {.Address = CDC0_NOTIFICATION_EPADDR, .Size = CDC_NOTIFICATION_EPSIZE, .Banks = 1},
			},
	},
	{
		.Config =
			{
				.ControlInterfaceNumber = INTERFACE_ID_CDC1_CCI,
				.DataINEndpoint         = {.Address = CDC1_TX_EPADDR, .Size = CDC_TXRX_EPSIZE, .Banks = CDC_TXRX_BANKS},
				.DataOUTEndpoint        = {.Address = CDC1_RX_EPADDR, .Size = CDC_TXRX_EPSIZE, .Banks = CDC_TXRX_BANKS},
				.NotificationEndpoint   = {.Address = CDC1_NOTIFICATION_EPADDR, .Size = CDC_NOTIFICATION_EPSIZE, .Banks = 1},
			},
	},
};

#define DEFAULT_LINE { .BaudRateBPS = 9600, .CharFormat = CDC_LINEENCODING_OneStopBit, \
                       .ParityType = CDC_PARITY_None, .DataBits = 8 }

static bridge_t bridge[NUM_PORTS] =
{
	{ .uart = &hwuart_port,   .line = DEFAULT_LINE },
	{ .uart = &softuart_port, .line = DEFAULT_LINE },
};

static volatile bool enter_bootloader;
static uint32_t      led_tx_until, led_rx_until;

/* ---- LEDs (Pro Micro: RX LED on PB0, TX LED on PD5, both active low) ---- */

static void leds_init(void)
{
	DDRB  |= _BV(PB0);
	PORTB |= _BV(PB0);
	DDRD  |= _BV(PD5);
	PORTD |= _BV(PD5);
}

static void leds_task(uint32_t now)
{
	if (clock_reached(now, led_rx_until)) PORTB |= _BV(PB0); else PORTB &= ~_BV(PB0);
	if (clock_reached(now, led_tx_until)) PORTD |= _BV(PD5); else PORTD &= ~_BV(PD5);
}

/* ---- Flow control and modem control lines ---- */

/* One init / service function pair per port, generated for its pins. */
#define MODEM_FUNCTIONS(n)                                                     \
	static void modem##n##_init(void)                                          \
	{                                                                          \
		GPIO_SET(P##n##_RTS_PIN);              /* deasserted */                \
		GPIO_OUTPUT(P##n##_RTS_PIN);                                           \
		if (P##n##_HW_FLOW)                                                    \
		  GPIO_INPUT_PU(P##n##_CTS_PIN);                                       \
		if (MODEM_LINES)                                                       \
		{                                                                      \
			GPIO_SET(P##n##_DTR_PIN);                                          \
			GPIO_OUTPUT(P##n##_DTR_PIN);                                       \
			GPIO_INPUT_PU(P##n##_DSR_PIN);                                     \
			GPIO_INPUT_PU(P##n##_DCD_PIN);                                     \
			GPIO_INPUT_PU(P##n##_RI_PIN);                                      \
		}                                                                      \
	}                                                                          \
	                                                                           \
	static void modem##n##_task(void)                                          \
	{                                                                          \
		bridge_t* b    = &bridge[n];                                           \
		uint16_t  host = cdc[n].State.ControlLineStates.HostToDevice;          \
		bool      rts  = host & CDC_CONTROL_LINE_OUT_RTS;                      \
		                                                                       \
		if (P##n##_HW_FLOW == FLOW_AUTO)                                       \
		{                                                                      \
			/* obey CTS once seen asserted while the port is open */           \
			bool open = host & CDC_CONTROL_LINE_OUT_DTR;                       \
			if (!open)                                                         \
			  b->cts_armed = false;                                            \
			else if (!b->cts_armed && GPIO_ASSERTED(P##n##_CTS_PIN))           \
			  b->cts_armed = true;                                             \
			b->uart->cts_arm(b->cts_armed);                                    \
		}                                                                      \
		if (P##n##_HW_FLOW)                                                    \
		{                                                                      \
			uint8_t room = ring_free(b->uart->rx);                             \
			if (room < RTS_OFF_FREE)                                           \
			  b->rx_full = true;                                               \
			else if (room >= RTS_ON_FREE)                                      \
			  b->rx_full = false;                                              \
			rts = rts && !b->rx_full;                                          \
		}                                                                      \
		GPIO_ASSERT(P##n##_RTS_PIN, rts);                                      \
		                                                                       \
		if (MODEM_LINES)                                                       \
		{                                                                      \
			GPIO_ASSERT(P##n##_DTR_PIN, host & CDC_CONTROL_LINE_OUT_DTR);      \
			                                                                   \
			uint16_t st = b->serial_state &                                    \
			              ~(CDC_CONTROL_LINE_IN_DCD | CDC_CONTROL_LINE_IN_DSR);  \
			if (GPIO_ASSERTED(P##n##_DCD_PIN))                                 \
			  st |= CDC_CONTROL_LINE_IN_DCD;                                   \
			if (GPIO_ASSERTED(P##n##_DSR_PIN))                                 \
			  st |= CDC_CONTROL_LINE_IN_DSR;                                   \
			bool ri = GPIO_ASSERTED(P##n##_RI_PIN);                            \
			if (ri && !b->ri_last)                                             \
			  st |= CDC_CONTROL_LINE_IN_RING;    /* reported per ring */       \
			b->ri_last      = ri;                                              \
			b->serial_state = st;                                              \
		}                                                                      \
	}

MODEM_FUNCTIONS(0)
MODEM_FUNCTIONS(1)

/* ---- Bootloader ---- */

/* Jump to the Caterina bootloader by planting its magic key and letting the
 * watchdog reset us (same mechanism as the Arduino core). */
static void reboot_to_bootloader(void)
{
	USB_Disable();
	cli();

	/* Newer Caterina builds (signature 0xDCFB at the top of flash) look for
	 * the key at the top of RAM; older ones at 0x0800. Nothing touches RAM
	 * after this point, so clobbering whatever lives there is harmless. */
	uint16_t* key = (uint16_t*)0x0800;
	if (pgm_read_word(FLASHEND - 1) == 0xDCFB)
	  key = (uint16_t*)(RAMEND - 1);
	*key = 0x7777;

	wdt_enable(WDTO_15MS);
	for (;;);
}

/* ---- Data pumps ---- */

/* Host -> UART: move what fits from the OUT endpoint into the tx ring. */
static void usb_to_uart(uint8_t port)
{
	ring_t* r = bridge[port].uart->tx;

	Endpoint_SelectEndpoint(cdc[port].Config.DataOUTEndpoint.Address);
	if (!Endpoint_IsOUTReceived())
	  return;

	uint8_t n    = Endpoint_BytesInEndpoint();
	uint8_t room = ring_free(r);
	if (n > room)
	  n = room;

	if (n)
	{
		uint8_t h = r->head;
		while (n--)
		  r->buf[h++ & RING_MASK] = Endpoint_Read_8();
		r->head = h;
		led_tx_until = clock_ticks() + LED_PULSE;
	}

	if (!Endpoint_BytesInEndpoint())
	  Endpoint_ClearOUT();
}

/* UART -> host: send what is in the rx ring as one packet when the IN bank
 * is free. A full-size packet is followed by a ZLP once the ring runs dry so
 * the host doesn't sit on a partially filled transfer. */
static void uart_to_usb(uint8_t port)
{
	bridge_t* b = &bridge[port];
	ring_t*   r = b->uart->rx;

	Endpoint_SelectEndpoint(cdc[port].Config.DataINEndpoint.Address);
	if (!Endpoint_IsINReady())
	  return;

	uint8_t n = ring_count(r);
	if (n == 0)
	{
		if (b->in_last_full)
		{
			Endpoint_ClearIN();
			b->in_last_full = false;
		}
		return;
	}

	if (n > CDC_TXRX_EPSIZE)
	  n = CDC_TXRX_EPSIZE;
	b->in_last_full = (n == CDC_TXRX_EPSIZE);

	uint8_t t = r->tail;
	for (uint8_t i = n; i; i--)
	  Endpoint_Write_8(r->buf[t++ & RING_MASK]);
	r->tail = t;
	Endpoint_ClearIN();

	led_rx_until = clock_ticks() + LED_PULSE;
}

/* Send a SERIAL_STATE notification if something changed and the interrupt
 * endpoint is free. */
static void notify_task(uint8_t port)
{
	bridge_t* b = &bridge[port];
	uint16_t  state = b->serial_state;

	if (!(state & SERIAL_STATE_EVENTS) && state == b->serial_state_sent)
	  return;

	Endpoint_SelectEndpoint(cdc[port].Config.NotificationEndpoint.Address);
	if (!Endpoint_IsINReady())
	  return;

	Endpoint_Write_8(REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE);
	Endpoint_Write_8(CDC_NOTIF_SerialState);
	Endpoint_Write_16_LE(0);
	Endpoint_Write_16_LE(cdc[port].Config.ControlInterfaceNumber);
	Endpoint_Write_16_LE(sizeof(uint16_t));
	Endpoint_Write_16_LE(state);
	Endpoint_ClearIN();

	b->serial_state      = state & ~SERIAL_STATE_EVENTS;
	b->serial_state_sent = b->serial_state;
}

/* ---- Per port service ---- */

static void bridge_task(uint8_t port, uint32_t now)
{
	bridge_t*          b = &bridge[port];
	const uart_port_t* u = b->uart;

	u->task();

	if (port == 0)
	  modem0_task();
	else
	  modem1_task();

	uint8_t err;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		err = *u->errors;
		*u->errors = 0;
	}
	b->serial_state |= err;

	/* break / line coding changes */
	if ((b->pending & PEND_BREAK_OFF) ||
	    (b->break_active && b->break_timed && clock_reached(now, b->break_end)))
	{
		b->pending &= ~PEND_BREAK_OFF;
		if (b->break_active)
		  u->set_break(false);
		b->break_active = false;
	}

	if ((b->pending & (PEND_CONFIG | PEND_BREAK_ON)) && (b->break_active || u->tx_idle()))
	{
		if (b->pending & PEND_CONFIG)
		{
			u->configure(&b->line);
			if (b->break_active)
			  u->set_break(true);
		}
		if (b->pending & PEND_BREAK_ON)
		{
			if (!b->break_active)
			  u->set_break(true);
			b->break_active = true;
			b->break_end    = now + CLOCK_MS(b->break_ms);
		}
		b->pending &= ~(PEND_CONFIG | PEND_BREAK_ON);
	}

	if (USB_DeviceState != DEVICE_STATE_Configured)
	  return;

	/* new host data is held back while a change is waiting for TX to drain */
	if (!b->pending)
	  usb_to_uart(port);
	if (!b->break_active)
	  u->tx_start();
	uart_to_usb(port);
	notify_task(port);
}

int main(void)
{
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);

	/* free PF4..PF7 from JTAG (must be written twice within 4 cycles) */
	MCUCR = _BV(JTD);
	MCUCR = _BV(JTD);

	clock_init();
	leds_init();
	modem0_init();
	modem1_init();
	hwuart_init();
	softuart_init();
	USB_Init();
	GlobalInterruptEnable();

	for (;;)
	{
		USB_USBTask();

		uint32_t now = clock_ticks();
		for (uint8_t i = 0; i < NUM_PORTS; i++)
		  bridge_task(i, now);
		leds_task(now);

		if (enter_bootloader)
		  reboot_to_bootloader();
	}
}

/* ---- LUFA events ---- */

static int8_t port_of(const USB_ClassInfo_CDC_Device_t* const c)
{
	return (c == &cdc[0]) ? 0 : 1;
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
	for (uint8_t i = 0; i < NUM_PORTS; i++)
	{
		CDC_Device_ConfigureEndpoints(&cdc[i]);
		cdc[i].State.LineEncoding = bridge[i].line;   /* report what is in effect */
		bridge[i].in_last_full = false;
		ring_flush(bridge[i].uart->rx);
	}
}

void EVENT_USB_Device_ControlRequest(void)
{
	/* SEND_BREAK is handled here rather than by LUFA's class driver, which
	 * truncates the 16-bit duration to 8 bits (0xFFFF = "until cleared"). */
	if (USB_ControlRequest.bRequest == CDC_REQ_SendBreak &&
	    USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
	{
		for (uint8_t i = 0; i < NUM_PORTS; i++)
		{
			if (USB_ControlRequest.wIndex != cdc[i].Config.ControlInterfaceNumber)
			  continue;

			Endpoint_ClearSETUP();
			Endpoint_ClearStatusStage();

			/* the host waited for its writes to complete before asking for
			 * the break, so what sits in the OUT bank goes out first */
			usb_to_uart(i);
			Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);

			bridge_t* b = &bridge[i];
			uint16_t  ms = USB_ControlRequest.wValue;
			if (ms == 0)
			{
				b->pending &= ~PEND_BREAK_ON;
				b->pending |= PEND_BREAK_OFF;
			}
			else
			{
				b->pending    &= ~PEND_BREAK_OFF;
				b->pending    |= PEND_BREAK_ON;
				b->break_timed = (ms != 0xFFFF);
				b->break_ms    = ms;
			}
			return;
		}
	}

	for (uint8_t i = 0; i < NUM_PORTS; i++)
	  CDC_Device_ProcessControlRequest(&cdc[i]);
}

void EVENT_CDC_Device_LineEncodingChanged(USB_ClassInfo_CDC_Device_t* const c)
{
	uint8_t port = port_of(c);

	/* Data already in the OUT bank was written before the change (the
	 * host completed it before sending SET_LINE_CODING), so queue it to be
	 * sent with the old settings. */
	usb_to_uart(port);
	Endpoint_SelectEndpoint(ENDPOINT_CONTROLEP);

	bridge[port].line     = c->State.LineEncoding;
	bridge[port].pending |= PEND_CONFIG;
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const c)
{
#if BOOTLOADER_1200BAUD_TOUCH
	if (c == &cdc[0] && c->State.LineEncoding.BaudRateBPS == 1200 &&
	    !(c->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR))
	  enter_bootloader = true;
#endif
}
