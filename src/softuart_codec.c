/*
 * Soft UART framing - see softuart_codec.h.
 */
#include "softuart_codec.h"

static uint8_t parity_bit(const su_cfg_t* c, uint8_t data)
{
	uint8_t p = data & (uint8_t)((1 << c->data_bits) - 1);
	p ^= p >> 4;
	p ^= p >> 2;
	p ^= p >> 1;
	p &= 1;

	switch (c->parity)
	{
		case SU_PARITY_ODD:  return !p;
		case SU_PARITY_EVEN: return p;
		case SU_PARITY_MARK: return 1;
		default:             return 0;
	}
}

void su_cfg_init(su_cfg_t* c, uint32_t timer_hz, uint32_t baud,
                 uint8_t data_bits, uint8_t parity, uint8_t stop)
{
	if (data_bits < 5 || data_bits > 8)
	  data_bits = 8;
	if (parity > SU_PARITY_SPACE)
	  parity = SU_PARITY_NONE;
	if (stop > SU_STOP_2)
	  stop = SU_STOP_1;

	c->data_bits = data_bits;
	c->parity    = parity;
	c->nbits     = 1 + data_bits + (parity != SU_PARITY_NONE) + 1;

	/* Offsets are computed exactly to 1/256 tick from the integer and
	 * fractional parts of timer_hz / baud (no 64-bit arithmetic needed for
	 * the rates the soft UART supports). */
	uint32_t q = timer_hz / baud;
	uint32_t r = timer_hz % baud;
	#define HALF_BITS_Q8(n) ((n) * q * 128 + ((n) * r * 128 + baud / 2) / baud)

	for (uint8_t i = 0; i < c->nbits; i++)
	{
		c->bit_start[i] = (uint16_t)((HALF_BITS_Q8(2u * i) + 128) >> 8);
		c->sample[i]    = (uint16_t)((HALF_BITS_Q8(2u * i + 1) + 128) >> 8);
	}

	/* everything up to the stop bits, then 2, 3 or 4 half bits of stop */
	uint32_t frame_q8 = HALF_BITS_Q8(2u * (c->nbits - 1) + 2 + stop);
	c->frame_len  = (uint16_t)(frame_q8 >> 8);
	c->frame_frac = (uint8_t)frame_q8;
	#undef HALF_BITS_Q8
}

void su_enc_reset(su_enc_t* e)
{
	e->tail = 0;
	e->frac = 128;   /* round frame starts to the nearest tick */
}

uint8_t su_encode(const su_cfg_t* c, su_enc_t* e, uint8_t data, su_edge_t* out)
{
	/* frame bits, LSB first: start (0), data, parity, stop (1) */
	uint16_t bits = (uint16_t)(data & ((1 << c->data_bits) - 1)) << 1;
	uint8_t  n    = 1 + c->data_bits;
	if (c->parity != SU_PARITY_NONE)
	  bits |= (uint16_t)parity_bit(c, data) << n++;
	bits |= (uint16_t)1 << n;

	uint8_t  count     = 0;
	uint8_t  level     = 1;            /* idle */
	uint16_t last_edge = 0;

	for (uint8_t i = 0; i < c->nbits; i++)
	{
		uint8_t b = (bits >> i) & 1;
		if (b == level)
		  continue;

		out[count].delta = count ? (uint16_t)(c->bit_start[i] - last_edge) : e->tail;
		out[count].level = b;
		out[count].start = (count == 0);
		count++;

		level     = b;
		last_edge = c->bit_start[i];
	}

	uint16_t frac = (uint16_t)e->frac + c->frame_frac;
	e->frac = (uint8_t)frac;
	e->tail = c->frame_len + (frac >> 8) - last_edge;
	return count;
}

void su_dec_reset(su_dec_t* d, uint8_t level)
{
	d->in_frame = 0;
	d->level    = level;
}

static void finish_frame(su_dec_t* d, const su_cfg_t* c, su_rx_cb_t cb)
{
	uint16_t bits  = d->bits;
	uint8_t  data  = (uint8_t)(bits >> 1) & (uint8_t)((1 << c->data_bits) - 1);
	uint8_t  flags = 0;

	d->in_frame = 0;

	if (!(bits >> (c->nbits - 1)))
	  flags |= SU_RX_FRAMING;

	if (bits == 0)
	{
		/* line low for the whole frame, stop bit included */
		cb(0, SU_RX_BREAK);
		return;
	}

	if (c->parity != SU_PARITY_NONE &&
	    ((bits >> (1 + c->data_bits)) & 1) != parity_bit(c, data))
	  flags |= SU_RX_PARITY;

	cb(data, flags);
}

/* Take every pending sample that lies strictly before 'limit' using the
 * current line level. Works on 16-bit offsets from the start bit, which is
 * what keeps this cheap enough to run per edge on an 8-bit CPU. */
static void sample_until(su_dec_t* d, const su_cfg_t* c, uint32_t limit, su_rx_cb_t cb)
{
	if (!d->in_frame)
	  return;

	uint32_t rel32 = limit - d->start;
	if ((int32_t)rel32 <= 0)
	  return;
	uint16_t rel = (rel32 > 0xFFFF) ? 0xFFFF : (uint16_t)rel32;   /* frames are shorter than that */

	uint8_t idx = d->idx;
	if (idx == 0)
	{
		if (c->sample[0] >= rel)
		  return;
		if (d->level)
		{
			/* start bit gone by mid-bit: a glitch, not a frame */
			d->in_frame = 0;
			return;
		}
		idx = 1;
	}

	uint8_t n = c->nbits;
	if (d->level)
	{
		uint16_t mask = (uint16_t)1 << idx;
		uint16_t bits = d->bits;
		while (idx < n && c->sample[idx] < rel)
		{
			bits |= mask;
			mask <<= 1;
			idx++;
		}
		d->bits = bits;
	}
	else
	{
		while (idx < n && c->sample[idx] < rel)
		  idx++;
	}

	d->idx = idx;
	if (idx == n)
	  finish_frame(d, c, cb);
}

void su_dec_edge(su_dec_t* d, const su_cfg_t* c, uint32_t t, uint8_t level, su_rx_cb_t cb)
{
	sample_until(d, c, t, cb);

	d->level = level;
	if (!d->in_frame && !level)
	{
		d->in_frame = 1;
		d->start    = t;
		d->idx      = 0;
		d->bits     = 0;
	}
}

void su_dec_time(su_dec_t* d, const su_cfg_t* c, uint32_t now, su_rx_cb_t cb)
{
	sample_until(d, c, now, cb);
}
