/*
 * Coarse millisecond-ish time base: Timer0 free running at F_CPU/1024
 * (64 us per tick at 16 MHz), polled from the main loop - no interrupt, so it
 * never adds latency to the UART ISRs. clock_ticks() must be called at least
 * every 16 ms to not miss a Timer0 wrap; the main loop easily does that.
 */
#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

#define CLOCK_TICK_US     (1024000000UL / F_CPU)
#define CLOCK_MS(ms)      ((uint32_t)(ms) * 1000UL / CLOCK_TICK_US)

void     clock_init(void);
uint32_t clock_ticks(void);

/* true once 'deadline' (in clock ticks) has been reached */
static inline uint8_t clock_reached(uint32_t now, uint32_t deadline)
{
	return (int32_t)(now - deadline) >= 0;
}

#endif
