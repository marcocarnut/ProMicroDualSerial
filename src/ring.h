/*
 * Single-producer / single-consumer byte ring. Indices are free-running
 * 8-bit counters, so count = head - tail and the size must be a power of two
 * no larger than 128. Either side may be an ISR as long as each index is only
 * written by its own side.
 */
#ifndef RING_H
#define RING_H

#include <stdint.h>
#include <stdbool.h>

#define RING_SIZE 128
#define RING_MASK (RING_SIZE - 1)

typedef struct
{
	volatile uint8_t head;   /* written by the producer */
	volatile uint8_t tail;   /* written by the consumer */
	uint8_t          buf[RING_SIZE];
} ring_t;

static inline uint8_t ring_count(const ring_t* r) { return (uint8_t)(r->head - r->tail); }
static inline uint8_t ring_free(const ring_t* r)  { return RING_SIZE - ring_count(r); }
static inline bool    ring_empty(const ring_t* r) { return r->head == r->tail; }

/* Caller must have checked ring_free() first. */
static inline void ring_put(ring_t* r, uint8_t b)
{
	uint8_t h = r->head;
	r->buf[h & RING_MASK] = b;
	r->head = h + 1;
}

/* Caller must have checked !ring_empty() first. */
static inline uint8_t ring_get(ring_t* r)
{
	uint8_t t = r->tail;
	uint8_t b = r->buf[t & RING_MASK];
	r->tail = t + 1;
	return b;
}

/* Consumer side only (or with the consumer ISR masked). */
static inline void ring_flush(ring_t* r) { r->tail = r->head; }

#endif
