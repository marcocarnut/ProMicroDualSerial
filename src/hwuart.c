/*
 * Port 0: hardware USART1, interrupt driven.
 *
 * Features beyond what the USART does natively:
 *  - Mark / space parity: emulated with one extra data bit of fixed value
 *    (5..7 data bits use a 6..8 bit frame, 8 data bits use the 9-bit mode).
 *    Received frames whose extra bit has the wrong value are flagged as
 *    parity errors.
 *  - 1.5 stop bits: sent as 2 (receivers only check the first stop bit).
 *  - Break: TX line held low via the port pin with the USART transmitter off;
 *    received breaks (framing error with all-zero data) are reported as
 *    breaks, not as a 0x00 byte.
 *  - CTS flow control (P0_HW_FLOW, see config.h): checked before each byte
 *    is handed to the USART, so at most the byte in progress and one in
 *    UDR1 go out after CTS is deasserted.
 *
 * Limits at 16 MHz: 244 baud to 2 Mbaud. Requests outside are clamped.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "hwuart.h"
#include "config.h"
#include "gpio.h"

static volatile uint8_t cts_armed;

#if P0_HW_FLOW == FLOW_ON
#define CTS_BLOCKED() GPIO_READ(P0_CTS_PIN)    /* deasserted = high */
#elif P0_HW_FLOW == FLOW_AUTO
#define CTS_BLOCKED() (cts_armed && GPIO_READ(P0_CTS_PIN))
#else
#define CTS_BLOCKED() 0
#endif

static ring_t           tx_ring;
static ring_t           rx_ring;
static volatile uint8_t rx_errors;

static bool    tx_started;     /* a byte has been written to UDR1 since configure() */
static uint8_t tx_mask;        /* data bits to transmit (clears bits above the frame) */
static uint8_t tx_or;          /* fixed mark bit for emulated mark parity (<8 data bits) */
static uint8_t rx_emul;        /* mask of the extra (parity) data bit, 0 when it is RXB8 or unused */
static uint8_t rx_emul_flags;  /* EMUL_* below */

#define EMUL_ACTIVE   _BV(0)
#define EMUL_RXB8     _BV(1)
#define EMUL_MARK     _BV(2)

/*
 * The USART interrupts must not hold off the soft UART's timer interrupts
 * (see softuart.c), so each masks its own source and re-enables interrupts
 * within ~17 cycles. The common case is then handled in asm with three
 * more registers saved: RX of a byte without errors or emulated parity
 * into a ring with room (~70 cycles in total), UDRE with a byte to send
 * (~75). Anything else goes to the C body with the remaining call-clobbered
 * registers saved. The source is unmasked again on exit, except for UDRE
 * when there is nothing (more) to send.
 */
#define ISR_ENTER                                                          \
		"push r24"                  "\n\t"                           \
		"in   r24, __SREG__"        "\n\t"                           \
		"push r24"                  "\n\t"                           \
		"lds  r24, %[ucsrb]"        "\n\t"                           \
		"andi r24, %[off]"          "\n\t"                           \
		"sts  %[ucsrb], r24"        "\n\t"                           \
		"sei"                       "\n\t"                           \
		"push r25"                  "\n\t"                           \
		"push r30"                  "\n\t"                           \
		"push r31"                  "\n\t"

/* the rest of what a C function may clobber, around 'call' */
#define CALL_C(fn)                                                         \
		"push r0"                   "\n\t"                           \
		"push r1"                   "\n\t"                           \
		"clr  r1"                   "\n\t"                           \
		"push r18"                  "\n\t"                           \
		"push r19"                  "\n\t"                           \
		"push r20"                  "\n\t"                           \
		"push r21"                  "\n\t"                           \
		"push r22"                  "\n\t"                           \
		"push r23"                  "\n\t"                           \
		"push r26"                  "\n\t"                           \
		"push r27"                  "\n\t"                           \
		"call " #fn                 "\n\t"                           \
		"pop  r27"                  "\n\t"                           \
		"pop  r26"                  "\n\t"                           \
		"pop  r23"                  "\n\t"                           \
		"pop  r22"                  "\n\t"                           \
		"pop  r21"                  "\n\t"                           \
		"pop  r20"                  "\n\t"                           \
		"pop  r19"                  "\n\t"                           \
		"pop  r18"                  "\n\t"                           \
		"pop  r1"                   "\n\t"                           \
		"pop  r0"                   "\n\t"

#define ISR_EXIT_UNMASK                                                    \
		"pop  r31"                  "\n\t"                           \
		"pop  r30"                  "\n\t"                           \
		"pop  r25"                  "\n\t"                           \
		"cli"                       "\n\t"                           \
		"lds  r24, %[ucsrb]"        "\n\t"                           \
		"ori  r24, %[on]"           "\n\t"                           \
		"sts  %[ucsrb], r24"        "\n\t"                           \
		"pop  r24"                  "\n\t"                           \
		"out  __SREG__, r24"        "\n\t"                           \
		"pop  r24"                  "\n\t"                           \
		"reti"                      "\n\t"

#define ISR_EXIT_MASKED                                                    \
		"pop  r31"                  "\n\t"                           \
		"pop  r30"                  "\n\t"                           \
		"pop  r25"                  "\n\t"                           \
		"cli"                       "\n\t"                           \
		"pop  r24"                  "\n\t"                           \
		"out  __SREG__, r24"        "\n\t"                           \
		"pop  r24"                  "\n\t"                           \
		"reti"                      "\n\t"

void hwuart_rx_body(void) __attribute__((used));

ISR(USART1_RX_vect, ISR_NAKED)
{
	asm volatile(
		ISR_ENTER
		"lds  r25, %[ucsra]"        "\n\t"
		"andi r25, %[errs]"         "\n\t"
		"lds  r24, %[emul]"         "\n\t"
		"or   r25, r24"             "\n\t"
		"brne 8f"                   "\n\t"
		"lds  r30, %[head]"         "\n\t"
		"lds  r24, %[tail]"         "\n\t"
		"mov  r25, r30"             "\n\t"
		"sub  r25, r24"             "\n\t"
		"cpi  r25, %[size]"         "\n\t"
		"breq 8f"                   "\n\t"   /* ring full */
		"lds  r24, %[udr]"          "\n\t"
		"mov  r25, r30"             "\n\t"
		"inc  r25"                  "\n\t"
		"andi r30, %[mask]"         "\n\t"
		"ldi  r31, 0"               "\n\t"
		"subi r30, lo8(-(%[buf]))"  "\n\t"
		"sbci r31, hi8(-(%[buf]))"  "\n\t"
		"st   Z, r24"               "\n\t"
		"sts  %[head], r25"         "\n\t"
	"9:"
		ISR_EXIT_UNMASK
	"8:"	/* errors, emulated parity or a full ring */
		CALL_C(hwuart_rx_body)
		"rjmp 9b"                   "\n\t"
		::
		[ucsrb] "n" (_SFR_MEM_ADDR(UCSR1B)),
		[ucsra] "n" (_SFR_MEM_ADDR(UCSR1A)),
		[udr]   "n" (_SFR_MEM_ADDR(UDR1)),
		[off]   "M" (~_BV(RXCIE1) & 0xFF),
		[on]    "M" (_BV(RXCIE1)),
		[errs]  "M" (_BV(FE1) | _BV(DOR1) | _BV(UPE1)),
		[emul]  "i" (&rx_emul_flags),
		[head]  "i" (&rx_ring.head),
		[tail]  "i" (&rx_ring.tail),
		[buf]   "i" (rx_ring.buf),
		[size]  "M" (RING_SIZE),
		[mask]  "M" (RING_MASK)
	);
}

ISR(USART1_UDRE_vect, ISR_NAKED)
{
	asm volatile(
		ISR_ENTER
		"lds  r30, %[tail]"         "\n\t"
		"lds  r24, %[head]"         "\n\t"
		"cp   r30, r24"             "\n\t"
		"breq 7f"                   "\n\t"   /* nothing to send */
#if P0_HW_FLOW
		/* CTS is checked before each byte; one already in UDR1 still goes */
#if P0_HW_FLOW == FLOW_AUTO
		"lds  r24, %[armed]"        "\n\t"
		"tst  r24"                  "\n\t"
		"breq 1f"                   "\n\t"
#endif
		"sbic %[ctspin], %[ctsbit]" "\n\t"
		"rjmp 7f"                   "\n\t"
	"1:"
#endif
		"mov  r25, r30"             "\n\t"
		"inc  r25"                  "\n\t"
		"andi r30, %[mask]"         "\n\t"
		"ldi  r31, 0"               "\n\t"
		"subi r30, lo8(-(%[buf]))"  "\n\t"
		"sbci r31, hi8(-(%[buf]))"  "\n\t"
		"ld   r24, Z"               "\n\t"
		"lds  r30, %[txmask]"       "\n\t"
		"and  r24, r30"             "\n\t"
		"lds  r30, %[txor]"         "\n\t"
		"or   r24, r30"             "\n\t"
		"sts  %[udr], r24"          "\n\t"
		"sts  %[tail], r25"         "\n\t"
		/* clear TXC1 so tx_idle() can tell when this byte has left the wire */
		"lds  r24, %[ucsra]"        "\n\t"
		"andi r24, %[u2x]"          "\n\t"
		"ori  r24, %[txc]"          "\n\t"
		"sts  %[ucsra], r24"        "\n\t"
		ISR_EXIT_UNMASK
	"7:"
		ISR_EXIT_MASKED
		::
		[ucsrb]  "n" (_SFR_MEM_ADDR(UCSR1B)),
		[ucsra]  "n" (_SFR_MEM_ADDR(UCSR1A)),
		[udr]    "n" (_SFR_MEM_ADDR(UDR1)),
		[off]    "M" (~_BV(UDRIE1) & 0xFF),
		[on]     "M" (_BV(UDRIE1)),
		[u2x]    "M" (_BV(U2X1)),
		[txc]    "M" (_BV(TXC1)),
		[armed]  "i" (&cts_armed),
		[ctspin] "I" (_SFR_IO_ADDR(GPIO_PINREG(P0_CTS_PIN))),
		[ctsbit] "I" (GPIO_BIT(P0_CTS_PIN)),
		[head]   "i" (&tx_ring.head),
		[tail]   "i" (&tx_ring.tail),
		[buf]    "i" (tx_ring.buf),
		[mask]   "M" (RING_MASK),
		[txmask] "i" (&tx_mask),
		[txor]   "i" (&tx_or)
	);
}

/* RX, the uncommon cases (see above) */
void hwuart_rx_body(void)
{
	uint8_t status = UCSR1A;
	uint8_t ctrl   = UCSR1B;   /* RXB8 must be read before UDR1 */
	uint8_t data   = UDR1;

	if ((status & (_BV(FE1) | _BV(DOR1) | _BV(UPE1))) | rx_emul_flags)
	{
		uint8_t err   = 0;
		bool    extra = 0;

		if (rx_emul_flags & EMUL_ACTIVE)
		{
			extra = (rx_emul_flags & EMUL_RXB8) ? (ctrl & _BV(RXB81)) : (data & rx_emul);
			data &= ~rx_emul;
			if (!extra != !(rx_emul_flags & EMUL_MARK))
			  err |= CDC_CONTROL_LINE_IN_PARITYERROR;
		}

		if (status & _BV(DOR1))
		  err |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
		if (status & _BV(UPE1))
		  err |= CDC_CONTROL_LINE_IN_PARITYERROR;
		if (status & _BV(FE1))
		{
			if (data == 0 && !extra)
			{
				/* line held low for the whole frame: a break, not data */
				rx_errors |= CDC_CONTROL_LINE_IN_BREAK;
				return;
			}
			err |= CDC_CONTROL_LINE_IN_FRAMEERROR;
		}
		rx_errors |= err;
	}

	if (!ring_free(&rx_ring))
	{
		rx_errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
		return;
	}
	ring_put(&rx_ring, data);
}

/* Pick UBRR and U2X for the closest achievable rate. */
/* Best UBRR1 / U2X1 for a baud rate; returns the rate they give. */
static uint32_t calc_baud(uint32_t baud, uint16_t* ubrr_out, uint8_t* u2x_out)
{
	uint8_t  best_u2x  = 0;
	uint16_t best_ubrr = 0;
	uint32_t best_err  = UINT32_MAX;
	uint32_t best_rate = 0;

	if (baud == 0)
	  baud = 9600;

	for (uint8_t u2x = 0; u2x < 2; u2x++)
	{
		uint32_t div  = u2x ? 8 : 16;
		uint32_t ubrr = (F_CPU + div * baud / 2) / (div * baud);
		ubrr = ubrr ? ubrr - 1 : 0;
		if (ubrr > 4095)
		  ubrr = 4095;

		uint32_t actual = F_CPU / (div * (ubrr + 1));
		uint32_t err    = (actual > baud) ? actual - baud : baud - actual;
		if (err < best_err)   /* strict: prefer U2X=0 (better RX sampling) on ties */
		{
			best_err  = err;
			best_ubrr = ubrr;
			best_u2x  = u2x;
			best_rate = (F_CPU + div * (ubrr + 1) / 2) / (div * (ubrr + 1));
		}
	}

	*ubrr_out = best_ubrr;
	*u2x_out  = best_u2x;
	return best_rate;
}

static void set_baud(uint32_t baud)
{
	uint16_t ubrr;
	uint8_t  u2x;
	calc_baud(baud, &ubrr, &u2x);
	UBRR1  = ubrr;
	UCSR1A = u2x ? _BV(U2X1) : 0;
}

static void hwuart_effective(CDC_LineEncoding_t* le)
{
	uint16_t ubrr;
	uint8_t  u2x;
	le->BaudRateBPS = calc_baud(le->BaudRateBPS, &ubrr, &u2x);
	if (le->DataBits < 5 || le->DataBits > 8)
	  le->DataBits = 8;
	if (le->ParityType > CDC_PARITY_Space)
	  le->ParityType = CDC_PARITY_None;
	if (le->CharFormat != CDC_LINEENCODING_OneStopBit)
	  le->CharFormat = CDC_LINEENCODING_TwoStopBits;   /* 1.5 is sent as 2 */
}

static void hwuart_configure(const CDC_LineEncoding_t* le)
{
	uint8_t bits = le->DataBits;
	if (bits < 5 || bits > 8)
	  bits = 8;

	uint8_t ucsr1c = 0;
	uint8_t ucsr1b = _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1);
	uint8_t emul   = 0;
	uint8_t frame_bits = bits;

	switch (le->ParityType)
	{
		case CDC_PARITY_Odd:   ucsr1c |= _BV(UPM11) | _BV(UPM10); break;
		case CDC_PARITY_Even:  ucsr1c |= _BV(UPM11);              break;
		case CDC_PARITY_Mark:  emul = EMUL_ACTIVE | EMUL_MARK; frame_bits++; break;
		case CDC_PARITY_Space: emul = EMUL_ACTIVE;             frame_bits++; break;
		default: break;
	}

	if (le->CharFormat != CDC_LINEENCODING_OneStopBit)
	  ucsr1c |= _BV(USBS1);

	/* UCSZ: 5..8 bits = 0..3, 9 bits = 7 */
	if (frame_bits == 9)
	{
		ucsr1c |= _BV(UCSZ11) | _BV(UCSZ10);
		ucsr1b |= _BV(UCSZ12);
		emul   |= EMUL_RXB8;
		if (emul & EMUL_MARK)
		  ucsr1b |= _BV(TXB81);   /* sticky: every transmitted frame gets it */
	}
	else
	{
		ucsr1c |= (frame_bits - 5) << UCSZ10;
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		UCSR1B = 0;   /* also flushes the receiver */
		set_baud(le->BaudRateBPS);
		UCSR1C = ucsr1c;
		UCSR1B = ucsr1b;

		tx_mask       = (bits == 8) ? 0xFF : (uint8_t)((1 << bits) - 1);
		tx_or         = ((emul & EMUL_MARK) && bits < 8) ? (uint8_t)(1 << bits) : 0;
		rx_emul       = (emul && bits < 8) ? (uint8_t)(1 << bits) : 0;
		rx_emul_flags = emul;
		tx_started    = false;
		ring_flush(&tx_ring);
	}
}

static void hwuart_task(void)
{
}

static void hwuart_tx_start(void)
{
	if (ring_empty(&tx_ring) || CTS_BLOCKED())
	  return;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tx_started = true;
		UCSR1B |= _BV(UDRIE1);
	}
}

static bool hwuart_tx_idle(void)
{
	if (UCSR1B & _BV(UDRIE1))
	  return false;
	if (!ring_empty(&tx_ring) && !CTS_BLOCKED())
	  return false;
	return !tx_started || (UCSR1A & _BV(TXC1));
}

static void hwuart_set_break(bool on)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (on)
		{
			PORTD &= ~_BV(PD3);
			UCSR1B &= ~_BV(TXEN1);    /* pin falls back to PORTD3 = 0 */
		}
		else
		{
			PORTD |= _BV(PD3);
			UCSR1B |= _BV(TXEN1);
		}
	}
}

static void hwuart_cts_arm(bool on)
{
	cts_armed = on;
}

void hwuart_init(void)
{
	DDRD  |= _BV(PD3);            /* TX, idle high */
	PORTD |= _BV(PD3) | _BV(PD2); /* RX pull-up */

	static const CDC_LineEncoding_t def =
		{ .BaudRateBPS = 9600, .CharFormat = CDC_LINEENCODING_OneStopBit,
		  .ParityType = CDC_PARITY_None, .DataBits = 8 };
	hwuart_configure(&def);
}

const uart_port_t hwuart_port =
{
	.tx        = &tx_ring,
	.rx        = &rx_ring,
	.errors    = &rx_errors,
	.configure = hwuart_configure,
	.effective = hwuart_effective,
	.task      = hwuart_task,
	.tx_start  = hwuart_tx_start,
	.tx_idle   = hwuart_tx_idle,
	.set_break = hwuart_set_break,
	.cts_arm   = hwuart_cts_arm,
};
