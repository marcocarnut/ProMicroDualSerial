/*
 * simavr harness for sim_fw: feeds bytes into USART1, loops the soft UART's
 * TX pin (PB5) back into its RX/ICP pin (PD4), and checks that
 *  - the bytes coming back out of USART1 are the ones fed in,
 *  - the PB5 waveform, decoded independently here, carries the same bytes,
 *  - every PB5 edge lies within a few cycles of its ideal time relative to
 *    the start of its frame.
 *
 * usage: harness <fw.elf> <soft baud> <bits> <parity 0-4> <stop 0-2> <pattern> <count>
 *   pattern: random | 55 | 00 | ff | ascii
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include "sim_avr.h"
#include "sim_elf.h"
#include "avr_ioport.h"
#include "avr_uart.h"
#include "avr_timer.h"
#include "sim_cycle_timers.h"
#include "sim_interrupts.h"
#include <stdint.h>

#define F_CPU 16000000.0

static avr_t*     avr;
static avr_irq_t* pd4;

static uint8_t  sent[100000];
static int      nsent;
static uint8_t  back[100000];
static int      nback;
static int      uart_xoff;

/* recent event log, dumped when the first error flag rises (TRACE=1) */
#define NLOG 40000
static char     log_txt[NLOG][96];
static int      log_n;
static void logf_(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
static void logf_(const char* fmt, ...)
{
	va_list ap; va_start(ap, fmt);
	vsnprintf(log_txt[log_n % NLOG], sizeof log_txt[0], fmt, ap);
	va_end(ap);
	log_n++;
}
static void int_cb(struct avr_irq_t* irq, uint32_t value, void* param)
{
	logf_("%10llu  vector %2d %s", (unsigned long long)avr->cycle, (int)(intptr_t)param,
	      irq->irq == AVR_INT_IRQ_RUNNING ? (value ? "RUN" : "end") : (value ? "pending" : "unpend"));
}
static void dump_log(void)
{
	int start = log_n > NLOG ? log_n - NLOG : 0;
	for (int i = start; i < log_n; i++) printf("    %s\n", log_txt[i % NLOG]);
}


/* PB5 edge log */
static uint64_t edge_t[2000000];
static uint8_t  edge_v[2000000];
static int      nedges;

/*
 * simavr 1.6 does not model OC1A faithfully in normal mode: on every OCR1x
 * write and every overflow it drives the pin to the inverse of the COM1A
 * action ("PWM style" set-at-TOP / clear-at-TOP), which real silicon never
 * does. Those spurious drives are always the opposite of what the current
 * COM1A mode would do on a match, so they are filtered out here and OC1A
 * is rebuilt from the genuine compare matches only.
 */
#define TCCR1A_ADDR 0x80
static int txline = 1;

/*
 * The loopback is delayed a little (as the input synchroniser and noise
 * canceller would on silicon). Without it the capture interrupt always
 * becomes pending at the same instant as the compare interrupt and runs
 * first; its "TIFR1 = 1 << ICF1" then wipes OCF1A in simavr 1.6, which
 * stores TIFR writes as raw bytes instead of only clearing the 1 bits.
 */
#define LOOP_DELAY 20

static avr_cycle_count_t loop_cb(struct avr_t* a, avr_cycle_count_t when, void* param)
{
	logf_("%10llu  RX pin -> %d (ICR1 was %u, TCCR1B %02x)", (unsigned long long)avr->cycle, (int)(intptr_t)param,
	      avr->data[0x86] | (avr->data[0x87] << 8), avr->data[0x81]);
	avr_raise_irq(pd4, (int)(intptr_t)param);
	return 0;
}

static void oc1a_changed(struct avr_irq_t* irq, uint32_t value, void* param)
{
	int mode = (avr->data[TCCR1A_ADDR] >> 6) & 3;
	int v    = value & 1;
	static int dbg = -1;
	if (dbg < 0) dbg = getenv("OCDEBUG") ? atoi(getenv("OCDEBUG")) : 0;
	if (dbg > 0)
	{
		dbg--;
		printf("oc1a raise %d at %llu mode %d OCR1A %u GPIOR0 %02x TIMSK1 %02x\n", v,
		       (unsigned long long)avr->cycle, mode, avr->data[0x88] | (avr->data[0x89] << 8),
		       avr->data[0x3E], avr->data[0x6F]);
	}
	if (!((mode == 3 && v == 1) || (mode == 2 && v == 0)))
	  return;
	if (v == txline)
	  return;
	txline = v;
	logf_("%10llu  TX edge -> %d", (unsigned long long)avr->cycle, v);
	edge_t[nedges] = avr->cycle;
	edge_v[nedges] = v;
	nedges++;
	avr_cycle_timer_register(avr, LOOP_DELAY, loop_cb, (void*)(intptr_t)v);
}

static void uart_out(struct avr_irq_t* irq, uint32_t value, void* param)
{
	back[nback++] = value;
}

static void uart_xoff_cb(struct avr_irq_t* irq, uint32_t value, void* param) { uart_xoff = 1; }
static void uart_xon_cb(struct avr_irq_t* irq, uint32_t value, void* param) { uart_xoff = 0; }

static int err_pin;
static const char* const err_names[8] = { "any", "soft-framing", 0, 0, "soft-parity", "soft-overrun", "soft-break", "hw-usart" };
static void pf_changed(struct avr_irq_t* irq, uint32_t value, void* param)
{
	int bit = (int)(intptr_t)param;
	if (!(value & 1)) return;
	if (bit == 0) { err_pin = 1; if (getenv("TRACE")) dump_log(); }
	printf("  error flag %s at cycle %llu (frame ~%.0f)\n", err_names[bit], (unsigned long long)avr->cycle,
	       (avr->cycle - 200000.0) / (F_CPU / 115200 * 10));
}

int main(int argc, char** argv)
{
	if (argc < 8) { fprintf(stderr, "usage\n"); return 2; }
	double baud   = atof(argv[2]);
	int    bits   = atoi(argv[3]);
	int    parity = atoi(argv[4]);
	int    stop   = atoi(argv[5]);
	const char* pattern = argv[6];
	int    count  = atoi(argv[7]);

	elf_firmware_t fw;
	memset(&fw, 0, sizeof fw);
	if (elf_read_firmware(argv[1], &fw)) { fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }
	avr = avr_make_mcu_by_name("atmega32u4");
	avr_init(avr);
	avr_load_firmware(avr, &fw);
	avr->frequency = (uint32_t)F_CPU;
	avr->log = 0;

	pd4 = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('D'), 4);
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_TIMER_GETIRQ('1'), TIMER_IRQ_OUT_COMP + 0),
	                        oc1a_changed, NULL);
	for (int b = 0; b < 8; b++)
	  if (err_names[b])
	    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), b), pf_changed, (void*)(intptr_t)b);

	avr_irq_t* uin = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_INPUT);
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUTPUT), uart_out, NULL);
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUT_XOFF), uart_xoff_cb, NULL);
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('1'), UART_IRQ_OUT_XON), uart_xon_cb, NULL);
	avr_raise_irq(pd4, 1);   /* idle line until the firmware drives PB5 */
	if (getenv("TRACE"))
	{
		static const int vecs[] = { 16, 17, 25, 26 };
		for (int i = 0; i < 4; i++)
		  for (int k = 0; k < 2; k++)
		    avr_irq_register_notify(avr_get_interrupt_irq(avr, vecs[i]) + k, int_cb, (void*)(intptr_t)vecs[i]);
	}

	/* bytes to send */
	srand(1);
	for (int i = 0; i < count; i++)
	{
		uint8_t b;
		if (!strcmp(pattern, "55")) b = 0x55;
		else if (!strcmp(pattern, "00")) b = 0x00;
		else if (!strcmp(pattern, "ff")) b = 0xFF;
		else if (!strcmp(pattern, "ascii")) b = 32 + rand() % 95;
		else b = rand();
		sent[i] = b & ((1 << bits) - 1);
	}

	/* Let the firmware boot, then feed bytes at slightly below the soft
	 * UART's frame rate so its rings never overflow. */
	int    nb       = 1 + bits + (parity != 0);
	double frame    = (nb + 1 + 0.5 * stop) / baud * F_CPU;
	double interval = frame * 1.01;
	double next     = 200000;
	int    fed      = 0;
	uint64_t end    = 0;

	for (;;)
	{
		int state = avr_run(avr);
		if (state == cpu_Done || state == cpu_Crashed) { fprintf(stderr, "cpu stopped (%d)\n", state); return 2; }

		if (fed < count && avr->cycle >= next && !uart_xoff)
		{
			avr_raise_irq(uin, sent[fed++]);
			next += interval;
			if (fed == count) end = avr->cycle + (uint64_t)(frame * 300) + 2000000;
		}
		if (fed == count && avr->cycle >= end)
		  break;
	}
	nsent = count;
	if (getenv("STATE"))
	  printf("end state: cycle %llu OCR1A %u TCCR1A %02x TCCR1B %02x TIMSK1 %02x TIFR1 %02x GPIOR0 %02x GPIOR1(qtail) %u last TX edge %llu\n",
	         (unsigned long long)avr->cycle, avr->data[0x88] | (avr->data[0x89] << 8), avr->data[0x80], avr->data[0x81],
	         avr->data[0x6F], avr->data[0x36], avr->data[0x3E], avr->data[0x4A],
	         nedges ? (unsigned long long)edge_t[nedges - 1] : 0ULL);

	if (getenv("DUMP"))
	{
		int n = atoi(getenv("DUMP"));
		for (int i = 0; i < nedges && i < n; i++)
		  printf("edge %3d  t=%10llu  +%7.1f bits  -> %d\n", i, (unsigned long long)edge_t[i],
		         i ? (edge_t[i] - edge_t[i - 1]) / (F_CPU / baud) : 0.0, edge_v[i]);
		printf("sent: "); for (int i = 0; i < 8; i++) printf("%02x ", sent[i]); printf("\n");
	}

	/* 1. bytes back through USART1 */
	int bad = 0;
	for (int i = 0; i < nsent && i < nback; i++)
	  if (back[i] != sent[i]) { if (bad < 5) printf("  loopback byte %d: got %02x want %02x\n", i, back[i], sent[i]); bad++; }
	printf("loopback: sent %d, got back %d, %d mismatches, error pin %d\n", nsent, nback, bad, err_pin);

	/* 2 + 3. decode PB5 independently and measure edge timing */
	double bitc = F_CPU / baud;
	int    ndec = 0, dbad = 0;
	double worst = 0;
	int    e = 0;
	/* skip the initial high level */
	while (e < nedges && edge_v[e] != 0) e++;
	while (e < nedges)
	{
		/* edge e is a start bit */
		double s = (double)edge_t[e];
		int fb[12], k = 0;
		fb[k++] = 0;
		uint8_t v = sent[ndec];
		for (int i = 0; i < bits; i++) fb[k++] = (v >> i) & 1;
		if (parity)
		{
			int p = 0;
			for (int i = 0; i < bits; i++) p ^= (v >> i) & 1;
			fb[k++] = parity == 1 ? !p : parity == 2 ? p : parity == 3 ? 1 : 0;
		}
		fb[k++] = 1;

		/* walk the expected transitions and match them to logged edges */
		int level = 1, ok = 1;
		for (int i = 0; i < k; i++)
		{
			if (fb[i] == level) continue;
			level = fb[i];
			if (e >= nedges || edge_v[e] != level) { ok = 0; break; }
			double dev = fabs((double)edge_t[e] - (s + i * bitc));
			if (dev > worst) worst = dev;
			e++;
		}
		static int nshown;
		if (!ok) { if (nshown++ < 3) {
		             printf("  waveform frame %d does not match byte %02x; edges from its start:\n", ndec, v);
		             int e0 = e; while (e0 > 0 && (double)edge_t[e0] > s) e0--;
		             for (int j = e0; j < nedges && j < e0 + 14; j++)
		               printf("     %+9.2f bits -> %d\n", (edge_t[j] - s) / bitc, edge_v[j]);
		           }
		           dbad++;
		           while (e < nedges && edge_v[e] != 0) e++; }
		ndec++;
		/* next start bit must not come before the end of the stop bits */
		if (e < nedges && (double)edge_t[e] < s + (nb + 1 + 0.5 * stop) * bitc - 6)
		{ static int nearly; if (nearly++ < 3) { printf("  frame %d: next start bit %.1f cycles early; edges from its start:\n", ndec,
		                       s + (nb + 1 + 0.5 * stop) * bitc - edge_t[e]);
		             int e0 = e; while (e0 > 0 && (double)edge_t[e0] > s) e0--;
		             for (int j = e0; j <= e && j < nedges; j++)
		               printf("     %+9.2f bits -> %d\n", (edge_t[j] - s) / bitc, edge_v[j]); }
		  dbad++; }
	}
	printf("waveform: %d frames, %d bad, worst edge deviation %.1f cycles (%.1f%% of a bit)\n",
	       ndec, dbad, worst, 100 * worst / bitc);

	/* edges may be off by the encoder's rounding (1 timer tick) and the
	 * capture of frame starts (1 tick) */
	static const int presc[] = { 1, 8, 64, 256, 1024 };
	int cs = 0;
	while (cs < 4 && F_CPU / presc[cs] / baud * 12 > 65000) cs++;
	double tol = 2.0 * presc[cs] + 4;   /* + simavr instruction-granular timing */

	int fail = bad || nback != nsent || err_pin || dbad || ndec != nsent || worst > tol;
	printf("%s\n", fail ? "FAIL" : "PASS");
	return fail;
}
