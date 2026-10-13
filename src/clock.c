#include <avr/io.h>
#include "clock.h"

static uint8_t  last_tcnt;
static uint32_t ticks;

void clock_init(void)
{
	TCCR0A = 0;
	TCCR0B = _BV(CS02) | _BV(CS00);   /* clk/1024 */
	last_tcnt = TCNT0;
}

uint32_t clock_ticks(void)
{
	uint8_t now = TCNT0;
	ticks += (uint8_t)(now - last_tcnt);
	last_tcnt = now;
	return ticks;
}
