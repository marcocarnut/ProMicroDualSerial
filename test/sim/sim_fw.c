/*
 * Firmware for the simavr test (no USB): bytes received on USART1 are sent
 * out of the soft UART, and bytes received by the soft UART are sent out of
 * USART1. The harness wires soft TX (PB5) back to soft RX (PD4), so every
 * byte fed into USART1 should come back out of it, having gone through the
 * soft UART's encoder, compare ISR, capture ISR and decoder.
 *
 * Error flags are latched on port D/E pins for the harness to see. The soft
 * UART is built with CTS flow control (P1_HW_FLOW, CTS on PF7) so the
 * harness can exercise it; it holds CTS asserted unless testing it.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "clock.h"
#include "hwuart.h"
#include "softuart.h"

#ifndef SOFT_BAUD
#define SOFT_BAUD 115200
#endif
#ifndef SOFT_BITS
#define SOFT_BITS 8
#endif
#ifndef SOFT_PARITY
#define SOFT_PARITY 0
#endif
#ifndef SOFT_STOP
#define SOFT_STOP 0
#endif
#ifndef HW_BAUD
#define HW_BAUD 500000
#endif

static void move(ring_t* from, ring_t* to)
{
	while (!ring_empty(from) && ring_free(to))
	  ring_put(to, ring_get(from));
}

int main(void)
{
	DDRD |= _BV(PD0) | _BV(PD1) | _BV(PD5) | _BV(PD6) | _BV(PD7);
	DDRE |= _BV(PE6);
	PORTF |= _BV(PF7);        /* CTS input pull-up */

	clock_init();
	hwuart_init();
	softuart_init();

	CDC_LineEncoding_t hw = { .BaudRateBPS = HW_BAUD, .CharFormat = 0, .ParityType = 0, .DataBits = 8 };
	CDC_LineEncoding_t sw = { .BaudRateBPS = SOFT_BAUD, .CharFormat = SOFT_STOP,
	                          .ParityType = SOFT_PARITY, .DataBits = SOFT_BITS };
	hwuart_port.configure(&hw);
	softuart_port.configure(&sw);
	sei();

	DDRB |= _BV(PB6) | _BV(PB4);
	for (;;)
	{
		PINB = _BV(PB6);    /* toggles: one edge per main loop iteration */
		clock_ticks();
		hwuart_port.task();
		softuart_port.task();

		move(hwuart_port.rx, softuart_port.tx);
		softuart_port.tx_start();
		move(softuart_port.rx, hwuart_port.tx);
		hwuart_port.tx_start();

		/* PB4: "ready for more" - stands in for USB NAKing the host when the
		 * rings are full (the harness only feeds USART1 while it is high) */
		if (ring_free(hwuart_port.rx) > 64 && ring_free(softuart_port.tx) > 64)
		  PORTB |= _BV(PB4);
		else
		  PORTB &= ~_BV(PB4);

		uint8_t serr, herr;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			serr = *softuart_port.errors;
			herr = *hwuart_port.errors;
			*softuart_port.errors = 0;
			*hwuart_port.errors = 0;
		}
		/* PD0: any error, PD1: soft framing, PD5: soft parity, PD6: soft
		 * overrun, PD7: soft break, PE6: hardware USART error */
		if (serr | herr)                               PORTD |= _BV(PD0);
		if (serr & CDC_CONTROL_LINE_IN_FRAMEERROR)     PORTD |= _BV(PD1);
		if (serr & CDC_CONTROL_LINE_IN_PARITYERROR)    PORTD |= _BV(PD5);
		if (serr & CDC_CONTROL_LINE_IN_OVERRUNERROR)   PORTD |= _BV(PD6);
		if (serr & CDC_CONTROL_LINE_IN_BREAK)          PORTD |= _BV(PD7);
		if (herr)                                      PORTE |= _BV(PE6);
	}
}
