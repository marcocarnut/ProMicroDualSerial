/*
 * Sampling profiler - see profiler.h.
 */
#include "config.h"
#include "profiler.h"

#if PROFILER
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define BUCKET_SHIFT 4                       /* words: 32-byte buckets */
#define NBUCKETS     344                     /* the first 11 KB of flash */
#define NLATE        16                      /* lateness bins of 16 cycles */

static volatile uint16_t hist[NBUCKETS];
static volatile uint16_t late[NLATE];
static volatile uint32_t total;
static volatile bool     sampling;
static uint8_t           lfsr = 1;

static int16_t           rep_pos = -1;       /* reporting: next line; -1: not */

void profiler_sample(uint16_t pc) __attribute__((used));

/* The stub reads the interrupted PC (a big-endian word address under the
 * four registers it saved) and re-enables interrupts within ~25 cycles, so
 * it holds off the soft UART's timer ISRs no longer than the USART ones. */
ISR(TIMER1_COMPB_vect, ISR_NAKED)
{
	asm volatile(
		"push r24"              "\n\t"
		"in   r24, __SREG__"    "\n\t"
		"push r24"              "\n\t"
		"push r25"              "\n\t"
		"push r30"              "\n\t"
		"push r31"              "\n\t"
		"in   r30, __SP_L__"    "\n\t"
		"in   r31, __SP_H__"    "\n\t"
		"ldd  r24, Z+7"         "\n\t"
		"ldd  r25, Z+6"         "\n\t"
		"sei"                   "\n\t"
		"push r0"               "\n\t"
		"push r1"               "\n\t"
		"clr  r1"               "\n\t"
		"push r18"              "\n\t"
		"push r19"              "\n\t"
		"push r20"              "\n\t"
		"push r21"              "\n\t"
		"push r22"              "\n\t"
		"push r23"              "\n\t"
		"push r26"              "\n\t"
		"push r27"              "\n\t"
		"call profiler_sample"  "\n\t"
		"pop  r27"              "\n\t"
		"pop  r26"              "\n\t"
		"pop  r23"              "\n\t"
		"pop  r22"              "\n\t"
		"pop  r21"              "\n\t"
		"pop  r20"              "\n\t"
		"pop  r19"              "\n\t"
		"pop  r18"              "\n\t"
		"pop  r1"               "\n\t"
		"pop  r0"               "\n\t"
		"pop  r31"              "\n\t"
		"pop  r30"              "\n\t"
		"pop  r25"              "\n\t"
		"pop  r24"              "\n\t"
		"out  __SREG__, r24"    "\n\t"
		"pop  r24"              "\n\t"
		"reti"                  "\n\t"
	);
}

void profiler_sample(uint16_t pc)
{
	uint16_t now = TCNT1;
	uint16_t due = OCR1B;

	/* next sample in 2048..3071 cycles */
	lfsr = (lfsr >> 1) ^ (-(lfsr & 1) & 0xB8);
	OCR1B = due + 2048 + ((uint16_t)lfsr << 2);

	if (!sampling)
	  return;

	uint16_t b = pc >> BUCKET_SHIFT;
	if (b >= NBUCKETS)
	  b = NBUCKETS - 1;
	if (hist[b] != 0xFFFF)
	  hist[b]++;

	uint16_t l = (uint16_t)(now - due) >> 4;
	if (l >= NLATE)
	  l = NLATE - 1;
	if (late[l] != 0xFFFF)
	  late[l]++;
	total++;
}

/* Timer1 runs free at F_CPU at 115200 (the soft UART picks its
 * prescaler; at lower rates sampling and lateness are in coarser ticks). */
void profiler_init(void)
{
	OCR1B   = TCNT1 + 2048;
	TIFR1   = _BV(OCF1B);
	TIMSK1 |= _BV(OCIE1B);
	sampling = true;
}

void profiler_report(void)
{
	sampling = false;    /* until the report is out; it clears the counts */
	rep_pos  = 0;
}

static void put(ring_t* r, const char* s)
{
	while (*s)
	  ring_put(r, *s++);
}

static void put_num(ring_t* r, char tag, uint16_t a, uint16_t b)
{
	char buf[8];
	ring_put(r, tag);
	ring_put(r, ' ');
	put(r, utoa(a, buf, 16));
	ring_put(r, ' ');
	put(r, utoa(b, buf, 10));
	ring_put(r, '\n');
}

/* Lines: "N <total samples / 65536 hex> <total % 65536>", "P <bucket start
 * byte address, hex> <count>", "L <lateness bin> <count>", "S <counter>
 * <count>" (su_stats), then "E". */
void profiler_task(ring_t* r)
{
	if (rep_pos < 0)
	  return;

	while (ring_free(r) >= 20)
	{
		if (rep_pos == 0)
		{
			put_num(r, 'N', total >> 16, (uint16_t)total);
			total = 0;
		}
		else if (rep_pos <= NBUCKETS)
		{
			uint16_t i = rep_pos - 1;
			if (hist[i])
			  put_num(r, 'P', (uint16_t)i << (BUCKET_SHIFT + 1), hist[i]);
			hist[i] = 0;
		}
		else if (rep_pos <= NBUCKETS + NLATE)
		{
			uint8_t i = rep_pos - 1 - NBUCKETS;
			if (late[i])
			  put_num(r, 'L', i, late[i]);
			late[i] = 0;
		}
		else if (rep_pos <= NBUCKETS + NLATE + SU_STATS)
		{
			uint8_t i = rep_pos - 1 - NBUCKETS - NLATE;
			put_num(r, 'S', i, su_stats[i]);
			su_stats[i] = 0;
		}
		else
		{
			put(r, "E\n");
			rep_pos  = -1;
			sampling = true;
			return;
		}
		rep_pos++;
	}
}
#endif
