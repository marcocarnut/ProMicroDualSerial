/*
 * Unit tests for softuart_codec.c: the encoder is checked against an ideal
 * waveform, and the decoder against encoder output (optionally with baud
 * rate mismatch, edge jitter, idle gaps and injected line errors).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "softuart_codec.h"

static int failures, checks;

#define CHECK(cond, ...) do { checks++; if (!(cond)) { failures++; \
	if (failures < 30) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

/* ---- decoder output capture ---- */

#define MAXOUT 100000
static uint8_t out_data[MAXOUT], out_flags[MAXOUT];
static int     nout;

static void rx_cb(uint8_t data, uint8_t flags)
{
	if (nout < MAXOUT) { out_data[nout] = data; out_flags[nout] = flags; }
	nout++;
}

/* ---- line simulation: absolute edge list ---- */

typedef struct { uint32_t t; uint8_t level; } edge_t;
#define MAXEDGES 1000000
static edge_t edges[MAXEDGES];
static int    nedges;

static void add_edge(uint32_t t, uint8_t level)
{
	/* drop no-op transitions (can happen when splicing in errors) */
	if (nedges && edges[nedges - 1].level == level)
	  return;
	if (!nedges && level == 1)
	  return;
	edges[nedges].t = t;
	edges[nedges].level = level;
	nedges++;
}

static uint32_t rnd_state = 12345;
static uint32_t rnd(void)
{
	rnd_state ^= rnd_state << 13; rnd_state ^= rnd_state >> 17; rnd_state ^= rnd_state << 5;
	return rnd_state;
}

static int parity_of(uint8_t data, int bits, int parity)
{
	int p = 0;
	for (int i = 0; i < bits; i++) p ^= (data >> i) & 1;
	switch (parity) {
		case SU_PARITY_ODD: return !p;
		case SU_PARITY_EVEN: return p;
		case SU_PARITY_MARK: return 1;
		default: return 0;
	}
}

/* Encodes 'n' bytes back to back (with optional random idle gaps) starting
 * at t0 the way the firmware's TX ISR plays them: the first frame after idle
 * starts at an absolute time, later ones 'delta' after the previous edge. */
static uint32_t play_frames(const su_cfg_t* c, const uint8_t* data, int n, uint32_t t0,
                            int gaps, int jitter)
{
	uint32_t t = t0;          /* time of the last edge */
	su_enc_t enc;
	int      idle = 1;

	su_enc_reset(&enc);
	for (int i = 0; i < n; i++)
	{
		su_edge_t e[SU_MAX_EDGES];
		if (idle)
		  su_enc_reset(&enc);
		uint8_t   k = su_encode(c, &enc, data[i], e);

		CHECK(k >= 1 && k <= SU_MAX_EDGES, "edge count %d", k);
		CHECK(e[0].start && e[0].level == 0, "first edge must be a start bit");

		for (uint8_t j = 0; j < k; j++)
		{
			if (j == 0 && idle)
			  t += 1000 + (gaps ? rnd() % 50000 : 0);
			else
			  t += e[j].delta;
			int32_t jit = jitter ? (int32_t)(rnd() % (2 * jitter + 1)) - jitter : 0;
			add_edge(t + jit, e[j].level);
		}
		idle = gaps && (rnd() % 3 == 0);
		if (idle)
		  t += enc.tail;   /* let the stop bits finish before the idle gap */
	}
	return t + enc.tail;
}

static void decode_all(const su_cfg_t* c, uint32_t end)
{
	su_dec_t d;
	su_dec_reset(&d, 1);
	nout = 0;
	for (int i = 0; i < nedges; i++)
	{
		/* interleave time polls like the main loop does */
		su_dec_time(&d, c, edges[i].t - 1, rx_cb);
		su_dec_edge(&d, c, edges[i].t, edges[i].level, rx_cb);
	}
	su_dec_time(&d, c, end + 100000, rx_cb);
}

/* ---- tests ---- */

/* The encoder's edges must land within half a tick of the ideal times. */
static void test_encoder_timing(void)
{
	static const uint32_t bauds[] = { 300, 1200, 9600, 57600, 115200, 230400 };
	for (int b = 0; b < 6; b++)
	for (int bits = 5; bits <= 8; bits++)
	for (int par = 0; par <= 4; par++)
	for (int stop = 0; stop <= 2; stop++)
	{
		uint32_t hz = (bauds[b] < 3000) ? 250000 : 16000000;
		su_cfg_t c;
		su_cfg_init(&c, hz, bauds[b], bits, par, stop);
		double bit = (double)hz / bauds[b];

		/* several frames back to back: edge times are cumulative, so this
		 * also checks that no error builds up across frames */
		nedges = 0;
		uint8_t data[64];
		for (int i = 0; i < 64; i++) data[i] = rnd();
		play_frames(&c, data, 64, 0, 0, 0);

		/* ideal waveform */
		int    nb = 1 + bits + (par != 0);
		double stop_bits = 1 + 0.5 * stop;
		double frame = (nb + stop_bits) * bit;
		double t0 = 1000;
		int    e = 0, level = 1;
		for (int f = 0; f < 64; f++)
		{
			int fb[12], k = 0;
			fb[k++] = 0;
			for (int i = 0; i < bits; i++) fb[k++] = (data[f] >> i) & 1;
			if (par) fb[k++] = parity_of(data[f], bits, par);
			fb[k++] = 1;
			for (int i = 0; i < k; i++)
			{
				if (fb[i] == level) continue;
				level = fb[i];
				double ideal = t0 + f * frame + i * bit;
				CHECK(e < nedges, "missing edges");
				if (e >= nedges) return;
				CHECK(edges[e].level == level, "edge %d level", e);
				CHECK(fabs(edges[e].t - ideal) <= 1.0,
				      "baud %u %d%c%s: edge %d at %u, ideal %.2f", bauds[b], bits, "NOEMS"[par],
				      stop == 0 ? "1" : stop == 1 ? "1.5" : "2", e, edges[e].t, ideal);
				e++;
			}
		}
		CHECK(e == nedges, "extra edges");
	}
}

/* Round trip through encoder and decoder, all line codings. */
static void test_roundtrip(double mismatch, int jitter, int gaps, uint32_t t0)
{
	static const uint32_t bauds[] = { 50, 300, 2400, 9600, 38400, 115200, 230400 };
	for (int b = 0; b < 7; b++)
	for (int bits = 5; bits <= 8; bits++)
	for (int par = 0; par <= 4; par++)
	for (int stop = 0; stop <= 2; stop++)
	{
		uint32_t hz = bauds[b] < 100 ? 15625 : bauds[b] < 3000 ? 250000 : 16000000;
		su_cfg_t tx, rx;
		su_cfg_init(&tx, hz, bauds[b], bits, par, stop);
		su_cfg_init(&rx, hz, (uint32_t)(bauds[b] * (1.0 + mismatch)), bits, par, stop);

		uint8_t data[300];
		for (int i = 0; i < 300; i++) data[i] = rnd() & ((1 << bits) - 1);
		data[0] = 0x00; data[1] = 0xFF & ((1 << bits) - 1); data[2] = 0x55 & ((1 << bits) - 1); data[3] = 0xAA & ((1 << bits) - 1);

		nedges = 0;
		uint32_t end = play_frames(&tx, data, 300, t0, gaps, jitter);
		decode_all(&rx, end);

		CHECK(nout == 300, "baud %u %d%c%d mismatch %.3f jitter %d gaps %d t0 %u: got %d bytes",
		      bauds[b], bits, "NOEMS"[par], stop, mismatch, jitter, gaps, t0, nout);
		int bad = 0;
		for (int i = 0; i < 300 && i < nout; i++)
		  if (out_data[i] != data[i] || out_flags[i]) { if (!bad && failures < 3) printf("  byte %d: got %02x/%02x want %02x\n", i, out_data[i], out_flags[i], data[i]); bad++; }
		CHECK(bad == 0, "baud %u %d%c%d mismatch %.3f jitter %d: %d bad bytes",
		      bauds[b], bits, "NOEMS"[par], stop, mismatch, jitter, bad);
	}
}

/* Builds a frame waveform by hand so errors can be injected. */
static uint32_t put_frame(uint32_t t, double bit, const int* fb, int n)
{
	for (int i = 0; i < n; i++)
	  add_edge(t + (uint32_t)(i * bit + 0.5), fb[i]);
	return t + (uint32_t)(n * bit + 0.5);
}

static void test_errors(void)
{
	uint32_t hz = 16000000, baud = 115200;
	double   bit = (double)hz / baud;
	su_cfg_t c;
	su_cfg_init(&c, hz, baud, 8, SU_PARITY_EVEN, SU_STOP_1);

	/* good, parity error, framing error, good */
	int good[]  = { 0, 1,0,1,0,0,0,0,0, 1, 1 };   /* 0x05, even parity = 0... */
	good[9] = parity_of(0x05, 8, SU_PARITY_EVEN);
	int perr[11]; memcpy(perr, good, sizeof perr); perr[9] ^= 1;
	int ferr[11]; memcpy(ferr, good, sizeof ferr); ferr[10] = 0;

	nedges = 0;
	uint32_t t = 5000;
	t = put_frame(t, bit, good, 11) + 300;
	t = put_frame(t, bit, perr, 11) + 300;
	t = put_frame(t, bit, ferr, 11);
	add_edge(t + 500, 1);                          /* line recovers after the bad stop bit */
	t = put_frame(t + 1000, bit, good, 11);
	decode_all(&c, t);

	CHECK(nout == 4, "errors: got %d frames", nout);
	CHECK(out_data[0] == 0x05 && out_flags[0] == 0, "good frame: %02x %02x", out_data[0], out_flags[0]);
	CHECK(out_data[1] == 0x05 && out_flags[1] == SU_RX_PARITY, "parity: %02x %02x", out_data[1], out_flags[1]);
	CHECK(out_data[2] == 0x05 && out_flags[2] == SU_RX_FRAMING, "framing: %02x %02x", out_data[2], out_flags[2]);
	CHECK(out_data[3] == 0x05 && out_flags[3] == 0, "after framing: %02x %02x", out_data[3], out_flags[3]);

	/* break: line low for 5 frame times -> exactly one break, then data */
	nedges = 0;
	add_edge(1000, 0);
	add_edge(1000 + (uint32_t)(55 * bit), 1);
	t = put_frame(1000 + (uint32_t)(60 * bit), bit, good, 11);
	decode_all(&c, t);
	CHECK(nout == 2, "break: got %d frames", nout);
	CHECK(out_flags[0] == SU_RX_BREAK, "break flags %02x", out_flags[0]);
	CHECK(out_data[1] == 0x05 && out_flags[1] == 0, "after break: %02x %02x", out_data[1], out_flags[1]);

	/* glitch shorter than half a bit is not a start bit */
	nedges = 0;
	add_edge(1000, 0);
	add_edge(1000 + (uint32_t)(0.3 * bit), 1);
	t = put_frame(1000 + (uint32_t)(3 * bit), bit, good, 11);
	decode_all(&c, t);
	CHECK(nout == 1 && out_data[0] == 0x05 && out_flags[0] == 0, "glitch: %d frames", nout);

	/* a 0x00 byte with a good stop bit is data, not a break */
	nedges = 0;
	int zero[] = { 0, 0,0,0,0,0,0,0,0, 0, 1 };
	t = put_frame(1000, bit, zero, 11);
	decode_all(&c, t);
	CHECK(nout == 1 && out_data[0] == 0 && out_flags[0] == 0, "zero byte: %d frames flags %02x", nout, out_flags[0]);
}

int main(void)
{
	test_encoder_timing();
	test_roundtrip(0, 0, 0, 0);
	test_roundtrip(0, 0, 1, 0);
	test_roundtrip(0, 0, 1, 0xFFFF0000u);   /* 32-bit time wrap mid-stream */
	test_roundtrip(+0.025, 0, 0, 0);
	test_roundtrip(-0.025, 0, 1, 0);
	test_roundtrip(0, 5, 1, 0);             /* +-5 ticks of edge jitter */
	test_errors();

	printf("%d checks, %d failures\n", checks, failures);
	return failures != 0;
}
