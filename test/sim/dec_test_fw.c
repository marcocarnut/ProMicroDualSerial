/*
 * Runs on the (simulated) AVR and checks the asm codec against the reference
 * C one (softuart_codec.c built with SU_DEC_C and SU_ENC_C, as su_dec_run_c
 * and su_encode_c):
 *
 *  - encoder (softuart_enc.S): every byte value in every line coding, with
 *    the encoder state carried along and the queue position varied; the
 *    queues, encoder states and returned heads must match
 *  - decoder (softuart_dec.S): identical pseudo-random edge streams; every
 *    decoded frame and the final decoder state must match
 *
 * Result on pins: PD1 goes high when done, PD0 high on any mismatch.
 * GPIOR0/1/2 hold the number of the failing round (low byte; 255 for the
 * encoder, with the byte value in GPIOR2), the failing config, and the
 * number of decoder rounds run / 64.
 */
#include <avr/io.h>
#include <string.h>
#include "softuart_codec.h"

void su_dec_run_c(su_dec_t* d, const su_cfg_t* c, const uint8_t* ring, uint8_t tail,
                  uint8_t head, uint8_t lvl_next, uint16_t now);
uint8_t su_encode_c(const su_cfg_t* c, su_enc_t* e, uint8_t data, uint8_t* q, uint8_t head);

static uint8_t qa[256] __attribute__((aligned(256)));
static uint8_t qc[256] __attribute__((aligned(256)));

static uint8_t  out_d[2][64], out_f[2][64];
static uint8_t  nout[2];
static uint8_t  which;

void su_rx_frame(uint8_t data, uint8_t flags)
{
	uint8_t n = nout[which];
	if (n < 64)
	{
		out_d[which][n] = data;
		out_f[which][n] = flags;
	}
	nout[which] = n + 1;
}

static uint32_t rnd_state = 1;
static uint16_t rnd(void)
{
	rnd_state = rnd_state * 1103515245UL + 12345;
	return rnd_state >> 16;
}

static uint8_t ring[256];

static void fail(uint8_t round, uint8_t cfg)
{
	GPIOR0 = round;
	GPIOR1 = cfg;
	PORTD |= _BV(PD0) | _BV(PD1);
	for (;;);
}

int main(void)
{
	DDRD |= _BV(PD0) | _BV(PD1);

	static const uint32_t bauds[] = { 115200, 57600, 9600, 230400 };
	su_cfg_t cfg;
	uint16_t rounds = 0;

	for (uint8_t ci = 0; ci < 4 * 4 * 5 * 3; ci++)
	{
		uint8_t b = ci % 4, bits = 5 + (ci / 4) % 4, par = (ci / 16) % 5, stop = ci / 80;
		su_cfg_init(&cfg, 16000000UL, bauds[b], bits, par, stop);

		su_enc_t ea, ec;
		su_enc_reset(&ea);
		ea.tail = rnd();
		ea.frac = rnd();
		ec = ea;
		uint8_t data = 0;
		do
		{
			uint8_t h  = rnd() & 0xFC;
			uint8_t ha = su_encode(&cfg, &ea, data, qa, h);
			uint8_t hc = su_encode_c(&cfg, &ec, data, qc, h);
			if (ha != hc || memcmp(&ea, &ec, sizeof ea) || memcmp(qa, qc, sizeof qa))
			{
				GPIOR2 = data;
				fail(255, ci);
			}
		}
		while (++data);
	}

	for (uint8_t ci = 0; ci < 4 * 4 * 5 * 3; ci++)
	{
		uint8_t b = ci % 4, bits = 5 + (ci / 4) % 4, par = (ci / 16) % 5, stop = ci / 80;
		su_cfg_init(&cfg, 16000000UL, bauds[b], bits, par, stop);
		uint16_t bit = 16000000UL / bauds[b];

		su_dec_t da, dc;
		su_dec_reset(&da, 1);
		su_dec_reset(&dc, 1);
		uint16_t t    = rnd();
		uint8_t  head = rnd() & 0xFE;
		uint8_t  lvl  = 1;

		for (uint8_t round = 0; round < 40; round++, rounds++)
		{
			/* a batch of edges: mostly bit-time multiples with jitter,
			 * sometimes glitches and long gaps */
			uint8_t tail = head;
			uint8_t n = rnd() % 24;
			uint8_t first_lvl = !lvl;
			for (uint8_t i = 0; i < n; i++)
			{
				uint16_t r = rnd();
				uint16_t dt;
				if ((r & 15) == 0)      dt = 1 + (rnd() % (bit / 2));       /* glitch */
				else if ((r & 15) == 1) dt = bit * 12 + rnd() % 30000;      /* gap */
				else                    dt = bit * (1 + (r >> 4) % 9) + (rnd() % 9) - 4;
				t += dt;
				ring[head] = (uint8_t)t;
				ring[(uint8_t)(head + 1)] = t >> 8;
				head += 2;
				lvl ^= 1;
			}
			uint16_t now = t + rnd() % (bit * 4);

			nout[0] = nout[1] = 0;
			which = 0;
			su_dec_run(&da, &cfg, ring, tail, head, first_lvl, now);
			which = 1;
			su_dec_run_c(&dc, &cfg, ring, tail, head, first_lvl, now);

			if (nout[0] != nout[1] ||
			    memcmp(out_d[0], out_d[1], nout[0] < 64 ? nout[0] : 64) ||
			    memcmp(out_f[0], out_f[1], nout[0] < 64 ? nout[0] : 64) ||
			    memcmp(&da, &dc, sizeof da))
			  fail(round, ci);

			/* the decoder works in 16-bit time: let any frame in progress
			 * finish before a long gap could wrap the timer */
			t = now;
		}
	}

	GPIOR2 = rounds >> 6;
	PORTD |= _BV(PD1);
	for (;;);
}
