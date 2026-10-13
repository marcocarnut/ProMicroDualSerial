/*
 * Port 1: soft UART. RX = PD4 (ICP1, Pro Micro pin 4) on Timer1,
 * TX = PC6 (OC3A, Pro Micro pin 5) on Timer3.
 */
#ifndef SOFTUART_H
#define SOFTUART_H

#include "uart_port.h"

void softuart_init(void);

extern const uart_port_t softuart_port;

#endif
