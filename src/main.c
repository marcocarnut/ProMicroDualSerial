/*
 * ProMicroDualUART - dual USB CDC <-> UART bridge for the ATmega32U4.
 *
 * Milestone 1: two CDC-ACM interfaces that echo back whatever they receive.
 */
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <LUFA/Drivers/USB/USB.h>

#include "config.h"
#include "Descriptors.h"

#define NUM_PORTS 2

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

static volatile bool enter_bootloader;

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

/* Echo everything received on a port's OUT endpoint back on its IN endpoint. */
static void echo_task(USB_ClassInfo_CDC_Device_t* const c)
{
	uint8_t buf[CDC_TXRX_EPSIZE];
	uint8_t n = 0;

	Endpoint_SelectEndpoint(c->Config.DataINEndpoint.Address);
	if (!Endpoint_IsINReady())
	  return;

	Endpoint_SelectEndpoint(c->Config.DataOUTEndpoint.Address);
	if (!Endpoint_IsOUTReceived())
	  return;
	while (Endpoint_BytesInEndpoint() && n < sizeof(buf))
	  buf[n++] = Endpoint_Read_8();
	if (!Endpoint_BytesInEndpoint())
	  Endpoint_ClearOUT();

	Endpoint_SelectEndpoint(c->Config.DataINEndpoint.Address);
	for (uint8_t i = 0; i < n; i++)
	  Endpoint_Write_8(buf[i]);
	Endpoint_ClearIN();
}

int main(void)
{
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	clock_prescale_set(clock_div_1);

	USB_Init();
	GlobalInterruptEnable();

	for (;;)
	{
		USB_USBTask();

		if (USB_DeviceState == DEVICE_STATE_Configured)
		{
			for (uint8_t i = 0; i < NUM_PORTS; i++)
			  echo_task(&cdc[i]);
		}

		if (enter_bootloader)
		  reboot_to_bootloader();
	}
}

void EVENT_USB_Device_ConfigurationChanged(void)
{
	for (uint8_t i = 0; i < NUM_PORTS; i++)
	  CDC_Device_ConfigureEndpoints(&cdc[i]);
}

void EVENT_USB_Device_ControlRequest(void)
{
	for (uint8_t i = 0; i < NUM_PORTS; i++)
	  CDC_Device_ProcessControlRequest(&cdc[i]);
}

void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t* const c)
{
#if BOOTLOADER_1200BAUD_TOUCH
	if (c == &cdc[0] && c->State.LineEncoding.BaudRateBPS == 1200 &&
	    !(c->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR))
	  enter_bootloader = true;
#endif
}
