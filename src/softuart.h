/*
 * Port 1: soft UART on Timer1. RX = PD4 (ICP1, Pro Micro pin 4),
 * TX = PB5 (OC1A, Pro Micro pin 9).
 */
#ifndef SOFTUART_H
#define SOFTUART_H

#include "uart_port.h"

void softuart_init(void);

extern const uart_port_t softuart_port;

#endif
