/*
 * Sampling profiler for the real board (debug builds: make EXTRA=-DPROFILER=1).
 *
 * Timer1's compare B interrupts the CPU at pseudo-random intervals and counts where it
 * was in a histogram of 32-byte code buckets. The soft UART's timer ISRs can't
 * be interrupted, so a sample due while one runs lands after it; how late each
 * sample was is recorded too, which measures the time spent in them.
 * tools/fwprofile.py collects the results and maps them to functions.
 */
#ifndef PROFILER_H
#define PROFILER_H

#include "ring.h"

/* softuart_tx_start() counters: calls, RX backlog gate closed with data
 * waiting, batch used up with data waiting, tx ring empty, frame word
 * queue full, transmitter (re)started from idle; then soft RX overruns:
 * edge ring, rx ring */
#define SU_TX_STATS 8
extern volatile uint16_t su_tx_stats[SU_TX_STATS];

#if PROFILER
void profiler_init(void);
/* Begin dumping the results (and reset them) into 'r', in pieces. */
void profiler_report(void);
void profiler_task(ring_t* r);
#else
static inline void profiler_init(void) {}
static inline void profiler_report(void) {}
static inline void profiler_task(ring_t* r) { (void)r; }
#endif

#endif
