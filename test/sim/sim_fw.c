/*
 * Firmware for the simavr test (no USB): bytes received on USART1 are sent
 * out of the soft UART, and bytes received by the soft UART are sent out of
 * USART1. The harness wires soft TX (PB5) back to soft RX (PD4), so every
 * byte fed into USART1 should come back out of it, having gone through the
 * soft UART's encoder, compare ISR, capture ISR and decoder.
 *
 * Soft UART errors are latched on PF0 for the harness to see.
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
	DDRF |= _BV(PF0) | _BV(PF1) | _BV(PF4) | _BV(PF5) | _BV(PF6) | _BV(PF7);

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
		/* PF0: any error, PF1: soft framing, PF4: soft parity, PF5: soft
		 * overrun, PF6: soft break, PF7: hardware USART error */
		if (serr | herr)                               PORTF |= _BV(PF0);
		if (serr & CDC_CONTROL_LINE_IN_FRAMEERROR)     PORTF |= _BV(PF1);
		if (serr & CDC_CONTROL_LINE_IN_PARITYERROR)    PORTF |= _BV(PF4);
		if (serr & CDC_CONTROL_LINE_IN_OVERRUNERROR)   PORTF |= _BV(PF5);
		if (serr & CDC_CONTROL_LINE_IN_BREAK)          PORTF |= _BV(PF6);
		if (herr)                                      PORTF |= _BV(PF7);
	}
}
