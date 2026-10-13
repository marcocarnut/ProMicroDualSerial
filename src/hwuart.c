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
 *
 * Limits at 16 MHz: 245 baud to 2 Mbaud. Requests outside are clamped.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "hwuart.h"
#include "config.h"

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

ISR(USART1_RX_vect)
{
	uint8_t status = UCSR1A;
	uint8_t ctrl   = UCSR1B;   /* RXB8 must be read before UDR1 */
	uint8_t data   = UDR1;

	if (__builtin_expect((status & (_BV(FE1) | _BV(DOR1) | _BV(UPE1))) | rx_emul_flags, 0))
	{
		uint8_t err = 0;
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

	uint8_t h = rx_ring.head;
	if ((uint8_t)(h - rx_ring.tail) == RING_SIZE)
	{
		rx_errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
		return;
	}
	rx_ring.buf[h & RING_MASK] = data;
	rx_ring.head = h + 1;
}

ISR(USART1_UDRE_vect)
{
	uint8_t t = tx_ring.tail;
	if (t == tx_ring.head)
	{
		UCSR1B &= ~_BV(UDRIE1);
		return;
	}
	UDR1 = (tx_ring.buf[t & RING_MASK] & tx_mask) | tx_or;
	tx_ring.tail = t + 1;
	/* clear TXC1 so tx_idle() can tell when this byte has left the wire */
	UCSR1A = (UCSR1A & _BV(U2X1)) | _BV(TXC1);
}

/* Pick UBRR and U2X for the closest achievable rate. */
static void set_baud(uint32_t baud)
{
	uint8_t  best_u2x  = 0;
	uint16_t best_ubrr = 0;
	uint32_t best_err  = UINT32_MAX;

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
		}
	}

	UBRR1     = best_ubrr;
	UCSR1A    = best_u2x ? _BV(U2X1) : 0;
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
	if (ring_empty(&tx_ring))
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
	if (!ring_empty(&tx_ring))
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
	.task      = hwuart_task,
	.tx_start  = hwuart_tx_start,
	.tx_idle   = hwuart_tx_idle,
	.set_break = hwuart_set_break,
};
