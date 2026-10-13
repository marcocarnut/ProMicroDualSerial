/*
 * Interface between the USB bridge and a UART implementation.
 *
 * Data flows through two rings: the bridge produces into 'tx' (host -> line)
 * and consumes from 'rx' (line -> host). Line errors are accumulated in
 * 'errors' as CDC SERIAL_STATE bits (CDC_CONTROL_LINE_IN_BREAK, _FRAMEERROR,
 * _PARITYERROR, _OVERRUNERROR) and collected by the bridge with interrupts
 * masked.
 */
#ifndef UART_PORT_H
#define UART_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include <LUFA/Drivers/USB/USB.h>
#include "ring.h"

typedef struct
{
	ring_t*           tx;
	ring_t*           rx;
	volatile uint8_t* errors;

	/* Apply a new line coding. Only called when tx_idle() is true; any
	 * data still queued for transmission (only possible when held off by
	 * flow control) is discarded. */
	void (*configure)(const CDC_LineEncoding_t* le);

	/* Main loop service routine. */
	void (*task)(void);

	/* (Re)start transmitting whatever is in the tx ring. Not called while
	 * a break is being sent. */
	void (*tx_start)(void);

	/* True when nothing is being shifted out and nothing will be until
	 * tx_start() is called with more data or flow control releases. */
	bool (*tx_idle)(void);

	/* Hold the TX line in the spacing (low) state. Only called when
	 * tx_idle() is true. */
	void (*set_break)(bool on);
} uart_port_t;

#endif
