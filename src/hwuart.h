/*
 * Port 0: the ATmega32U4's USART1 (RX = PD2, TX = PD3).
 */
#ifndef HWUART_H
#define HWUART_H

#include "uart_port.h"

void hwuart_init(void);

extern const uart_port_t hwuart_port;

#endif
