/*
 * Soft UART framing - see softuart_codec.h.
 */
#include "softuart_codec.h"

/* The encoder and decoder are the soft UART's main loop hot spots; -Os
 * makes the decoder spill its state to the stack on every edge. */
#if defined(__AVR__)
#pragma GCC optimize ("O2")
#endif

static uint8_t parity_bit(const su_cfg_t* c, uint8_t data)
{
	uint8_t p = data & c->data_mask;
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

	c->sample[c->nbits] = 0xFFFF;
	c->last_sample = c->sample[c->nbits - 1];
	c->data_mask   = (uint8_t)((1 << data_bits) - 1);
	c->stop_mask   = (uint16_t)1 << (c->nbits - 1);
	c->parity_mask = (parity != SU_PARITY_NONE) ? (uint16_t)1 << (c->nbits - 2) : 0;

	c->enc_or  = (uint16_t)(0xFFFF << (data_bits + (parity != SU_PARITY_NONE)));
	c->enc_par = (parity != SU_PARITY_NONE) ? (uint16_t)1 << data_bits : 0;
	static const uint8_t par_lut[] = { 0, 1, 2, 3, 0 };   /* none, odd, even, mark, space */
	c->par_lut = par_lut[parity];

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

/* On the AVR, softuart_enc.S has the encoder in asm; this C version is the
 * reference, used on the host and in the simulator to check the asm one. */
#if !defined(__AVR__) || defined(SU_ENC_C)
#if defined(__AVR__)
#define su_encode su_encode_c       /* SU_ENC_C: C version next to the asm one, for testing */
#endif
static const uint16_t su_pow2[SU_MAX_BITS + 1] =
	{ 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048 };

uint8_t su_encode(const su_cfg_t* c, su_enc_t* e, uint8_t data, uint8_t* q, uint8_t h)
{
	uint8_t n = c->nbits;

	/* frame bits, LSB first: start (0), data, parity, stop (1) */
	uint16_t bits = (uint16_t)(data & (uint8_t)((1 << c->data_bits) - 1)) << 1;
	if (c->parity != SU_PARITY_NONE)
	  bits |= (uint16_t)parity_bit(c, data) << (n - 2);
	bits |= (uint16_t)1 << (n - 1);

	/* bit i set where the level changes entering bit i (idle before bit 0 is 1) */
	uint16_t trans = (bits ^ ((bits << 1) | 1)) & (su_pow2[n] - 1);

	/* the start bit */
	q[h]     = (uint8_t)e->tail;
	q[h + 1] = (uint8_t)(e->tail >> 8);
	q[h + 2] = SU_CTL_LOW | SU_CTL_START;
	h += 4;

	const uint16_t* bs   = &c->bit_start[1];
	uint16_t        last = 0;
	trans >>= 1;
	bits  >>= 1;
	while (trans)
	{
		if (trans & 1)
		{
			uint16_t delta = *bs - last;
			last     = *bs;
			q[h]     = (uint8_t)delta;
			q[h + 1] = (uint8_t)(delta >> 8);
			q[h + 2] = (bits & 1) ? SU_CTL_HIGH : SU_CTL_LOW;
			h += 4;
		}
		trans >>= 1;
		bits  >>= 1;
		bs++;
	}

	uint16_t frac = (uint16_t)e->frac + c->frame_frac;
	e->frac = (uint8_t)frac;
	e->tail = c->frame_len + (frac >> 8) - last;
	return h;
}
#undef su_encode
#endif


void su_dec_reset(su_dec_t* d, uint8_t level)
{
	d->in_frame = 0;
	d->level    = level;
}

void su_dec_finish_frame(const su_cfg_t* c, uint16_t bits)
{
	if (bits == 0)
	{
		/* line low for the whole frame, stop bit included */
		su_rx_frame(0, SU_RX_BREAK);
		return;
	}

	uint8_t data  = (uint8_t)(bits >> 1) & c->data_mask;
	uint8_t flags = (bits & c->stop_mask) ? 0 : SU_RX_FRAMING;
	if (c->parity_mask && !(bits & c->parity_mask) != !parity_bit(c, data))
	  flags |= SU_RX_PARITY;

	su_rx_frame(data, flags);
}

/*
 * Decoder inner loop. It runs once per received edge: it loads the
 * timestamp, and only if the next sample point has been passed walks the
 * sample table (ending at its 0xFFFF sentinel) to fill in the bits.
 *
 * On the AVR the same algorithm is implemented in softuart_dec.S (the
 * compiler can't keep this state in registers); this C version is the
 * reference, used on the host and in the simulator to check the asm one.
 */
#if !defined(__AVR__) || defined(SU_DEC_C)
#if defined(__AVR__)
#define su_dec_run su_dec_run_c     /* SU_DEC_C: C version next to the asm one, for testing */
#endif
void su_dec_run(su_dec_t* d, const su_cfg_t* c, const uint8_t* ring, uint8_t tail,
                uint8_t head, uint8_t lvl_next, uint16_t now)
{
	uint8_t         in_frame = d->in_frame;
	uint8_t         lvl      = d->level;
	const uint16_t* next     = d->next;
	uint16_t        mask     = d->mask;
	uint16_t        bits     = d->bits;
	uint16_t        start    = d->start;

	/* every edge, then 'now' as a final pseudo-edge that changes nothing */
	for (;;)
	{
		uint8_t  edge = (tail != head);
		uint16_t t    = edge ? (ring[tail] | ((uint16_t)ring[(uint8_t)(tail + 1)] << 8)) : now;

		if (in_frame)
		{
			uint16_t rel = t - start;
			if (*next < rel)
			{
				if (next == c->sample && lvl)
				  in_frame = 0;   /* start bit gone by mid-bit: a glitch */
				else
				{
					do
					{
						if (lvl)
						  bits |= mask;
						mask <<= 1;
						next++;
					} while (*next < rel);
					if (*next == 0xFFFF)
					{
						su_dec_finish_frame(c, bits);
						in_frame = 0;
					}
				}
			}
		}

		if (!edge)
		  break;
		tail += 2;

		lvl = lvl_next;
		lvl_next ^= 1;
		if (!in_frame && !lvl)
		{
			in_frame = 1;
			start    = t;
			next     = c->sample;
			mask     = 1;
			bits     = 0;
		}
	}

	d->in_frame = in_frame;
	d->level    = lvl;
	d->next     = next;
	d->mask     = mask;
	d->bits     = bits;
	d->start    = start;
}
#undef su_dec_run
#endif

#if defined(__AVR__)
/* layout the asm decoder (softuart_dec.S) relies on */
#include <stddef.h>
_Static_assert(offsetof(su_dec_t, in_frame) == 0 && offsetof(su_dec_t, level) == 1 &&
               offsetof(su_dec_t, next) == 2 && offsetof(su_dec_t, mask) == 4 &&
               offsetof(su_dec_t, bits) == 6 && offsetof(su_dec_t, start) == 8,
               "su_dec_t layout changed: update softuart_dec.S");
_Static_assert(offsetof(su_cfg_t, parity) == SU_CFG_PARITY_OFS &&
               offsetof(su_cfg_t, bit_start) == SU_CFG_BIT_START_OFS &&
               offsetof(su_cfg_t, sample) == SU_CFG_SAMPLE_OFS &&
               offsetof(su_cfg_t, frame_len) == SU_CFG_FRAME_LEN_OFS &&
               offsetof(su_cfg_t, frame_frac) == SU_CFG_FRAME_FRAC_OFS &&
               offsetof(su_cfg_t, data_mask) == SU_CFG_DATA_MASK_OFS &&
               offsetof(su_cfg_t, enc_or) == SU_CFG_ENC_OR_OFS &&
               offsetof(su_cfg_t, enc_par) == SU_CFG_ENC_PAR_OFS &&
               offsetof(su_cfg_t, par_lut) == SU_CFG_PAR_LUT_OFS,
               "su_cfg_t layout changed: update the SU_CFG_*_OFS offsets");
_Static_assert(offsetof(su_enc_t, tail) == SU_ENC_TAIL_OFS && offsetof(su_enc_t, frac) == SU_ENC_FRAC_OFS,
               "su_enc_t layout changed: update the SU_ENC_*_OFS offsets");
#endif
