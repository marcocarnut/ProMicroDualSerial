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
 * environment:
 *   LOAD=x        offer TX data at x times the line rate (default 0.99; the
 *                 firmware's "ready" pin holds it off like USB would)
 *   RXGEN=1       drive the soft RX pin from an independent full-rate stream
 *                 instead of looping TX back
 *   PROFILE=file  write a cycles-per-address profile (see profile.py)
 *   QSTAT=qhead,txring  (data addresses of su_q_head and softuart.c's
 *                 tx_ring, from avr-nm) sample the soft transmitter's
 *                 state, queue and tx ring every 500 cycles
 *   TRACE=1, DUMP=n, OCDEBUG=n, STATE=1, GAPS=n: debugging aids
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
static int      rxgen;

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
 * OC1A is rebuilt from the timer's compare output IRQ. simavr 1.6 also
 * drove that IRQ to the inverse of the COM1A action on every OCR1x write
 * and overflow (fixed in 1.8); such drives are the opposite of what the
 * current COM1A mode does on a match, so they are filtered out. Harmless
 * with newer simavr.
 */
#define TCCR1A_ADDR 0x80
static int txline = 1;

/*
 * The loopback is delayed a little, as the wiring plus input synchroniser
 * and noise canceller would be on silicon (it also sidesteps simavr 1.6
 * losing one of two interrupts that become pending in the same cycle).
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
	static uint64_t dbg_from;
	if (dbg < 0) { dbg = getenv("OCDEBUG") ? atoi(getenv("OCDEBUG")) : 0;
	               dbg_from = getenv("OCDEBUG_FROM") ? strtoull(getenv("OCDEBUG_FROM"), 0, 0) : 0; }
	if (dbg > 0 && avr->cycle >= dbg_from)
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
	if (!rxgen)
	  avr_cycle_timer_register(avr, LOOP_DELAY, loop_cb, (void*)(intptr_t)v);
}

static void uart_out(struct avr_irq_t* irq, uint32_t value, void* param)
{
	back[nback++] = value;
}

static void uart_xoff_cb(struct avr_irq_t* irq, uint32_t value, void* param) { uart_xoff = 1; }
static void uart_xon_cb(struct avr_irq_t* irq, uint32_t value, void* param) { uart_xoff = 0; }

static uint64_t loop_last, loop_max, loop_max_at, loop_hist[8];
static void dec_tick(struct avr_irq_t* irq, uint32_t value, void* param)
{
	logf_("%10llu  decoder run (TCNT1 ~%u)", (unsigned long long)avr->cycle, 0);
}

static void loop_tick(struct avr_irq_t* irq, uint32_t value, void* param)
{
	if (loop_last && avr->cycle > 300000)
	{
		uint64_t d = avr->cycle - loop_last;
		if (d > loop_max) { loop_max = d; loop_max_at = avr->cycle; }
		int b = d < 500 ? 0 : d < 1000 ? 1 : d < 2000 ? 2 : d < 4000 ? 3 : d < 8000 ? 4 : 5;
		loop_hist[b]++;
	}
	loop_last = avr->cycle;
}

/* RXGEN mode: an independent transmitter drives the soft UART's RX pin with
 * back-to-back frames at exactly the line rate, while the soft TX output is
 * only checked (not looped back). */
static uint8_t   rx_sent[100000];
static int       rx_nsent;
static uint64_t* gen_t;
static uint8_t*  gen_v;
static int       gen_n, gen_i;

static avr_cycle_count_t gen_cb(struct avr_t* a, avr_cycle_count_t when, void* param)
{
	avr_raise_irq(pd4, gen_v[gen_i]);
	gen_i++;
	return gen_i < gen_n ? gen_t[gen_i] : 0;
}

/* CTS_TOGGLE mode: CTS is deasserted for random stretches; at most the one
 * frame already committed may start after that */
static avr_irq_t* cts_irq;
static int        cts_toggle, cts_on = 1;
static uint64_t   cts_t[100000];
static uint8_t    cts_v[100000];
static int        ncts;

static void set_cts(int on)
{
	cts_on = on;
	cts_t[ncts] = avr->cycle;
	cts_v[ncts++] = on;
	avr_raise_irq(cts_irq, !on);
}

static uint64_t starts[100000];   /* TX frame start times, from the waveform check */
static int      nstarts;

static int fw_ready = 1;
static void ready_cb(struct avr_irq_t* irq, uint32_t value, void* param) { fw_ready = value & 1; }

static int err_pin;
static const char* const err_names[6] = { "any", "soft-framing", "soft-parity", "soft-overrun", "soft-break", "hw-usart" };
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
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 6), loop_tick, NULL);
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 4), ready_cb, NULL);
	if (getenv("TRACE"))
	  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 7), dec_tick, NULL);
	avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_TIMER_GETIRQ('1'), TIMER_IRQ_OUT_COMP + 0),
	                        oc1a_changed, NULL);
	{
		static const char ports[6] = { 'D', 'D', 'D', 'D', 'D', 'E' };
		static const int  bits_[6] = { 0, 1, 5, 6, 7, 6 };
		for (int b = 0; b < 6; b++)
		  avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(ports[b]), bits_[b]), pf_changed, (void*)(intptr_t)b);
	}
	/* soft UART CTS (PF7, active low): asserted, or toggled with CTS_TOGGLE */
	cts_irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('F'), 7);
	cts_toggle = getenv("CTS_TOGGLE") != NULL;

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
	srand(getenv("SEED") ? atoi(getenv("SEED")) : 1);
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

	rxgen = getenv("RXGEN") != NULL;
	if (rxgen)
	{
		/* back-to-back frames from cycle 250000, exact bit times */
		/* RXGEN_BAUD: generator rate if different, e.g. 117647 = what the
		 * ATmega's USART really sends when asked for 115200 */
		double gbaud = getenv("RXGEN_BAUD") ? atof(getenv("RXGEN_BAUD")) : baud;
		double bt = F_CPU / gbaud, t = 250000;
		int    nbits = 1 + bits + (parity != 0);
		rx_nsent = count;
		gen_t = malloc(sizeof(uint64_t) * count * 12);
		gen_v = malloc(count * 12);
		int lvl = 1;
		for (int i = 0; i < count; i++)
		{
			uint8_t b = rand() & ((1 << bits) - 1);
			rx_sent[i] = b;
			int fb[12], k = 0;
			fb[k++] = 0;
			for (int j = 0; j < bits; j++) fb[k++] = (b >> j) & 1;
			if (parity)
			{
				int p = 0;
				for (int j = 0; j < bits; j++) p ^= (b >> j) & 1;
				fb[k++] = parity == 1 ? !p : parity == 2 ? p : parity == 3 ? 1 : 0;
			}
			fb[k++] = 1;
			for (int j = 0; j < k; j++)
			  if (fb[j] != lvl) { lvl = fb[j]; gen_t[gen_n] = (uint64_t)(t + j * bt + 0.5); gen_v[gen_n++] = lvl; }
			t += (nbits + 1 + 0.5 * stop) * bt;
		}
		/* cycle timers take a delay; convert absolute times to the callback's return (absolute) */
		avr_cycle_timer_register(avr, gen_t[0], gen_cb, NULL);
	}

	/* Let the firmware boot, then feed bytes at slightly below the soft
	 * UART's frame rate so its rings never overflow. */
	int    nb       = 1 + bits + (parity != 0);
	double frame    = (nb + 1 + 0.5 * stop) / baud * F_CPU;
	double load     = getenv("LOAD") ? atof(getenv("LOAD")) : 0.99;   /* fraction of the line rate fed */
	double interval = load > 0 ? frame / load : 0;
	if (load <= 0) count = 0;     /* RX only */
	double next     = 200000;
	int    fed      = 0;
	uint64_t first_fed = 0;   /* for the throughput figure */
	uint64_t end    = 0;

	static uint32_t prof[16384];          /* cycles per word address */
	int      profiling = getenv("PROFILE") != NULL;
	uint64_t prof_from = getenv("PROFILE_FROM") ? strtoull(getenv("PROFILE_FROM"), 0, 0) : 300000;
	for (;;)
	{
		uint32_t pc = avr->pc;
		uint64_t c0 = avr->cycle;
		/* PC hooks (byte addresses): log decoder runs and decoded frames
		 * without touching the firmware's timing */
		static uint32_t pc_dec = 1, pc_rx = 1;
		if (pc_dec == 1)
		{
			pc_dec = getenv("PC_DEC") ? strtoul(getenv("PC_DEC"), 0, 16) : 0;
			pc_rx  = getenv("PC_RX") ? strtoul(getenv("PC_RX"), 0, 16) : 0;
		}
		if (pc_dec && pc == pc_dec)
		  logf_("%10llu  su_dec_run tail %u head %u lvl %u now %u", (unsigned long long)avr->cycle,
		        avr->data[18], avr->data[16], avr->data[14], avr->data[12] | (avr->data[13] << 8));
		if (pc_rx && pc == pc_rx)
		  logf_("%10llu  su_rx_frame %02x flags %02x", (unsigned long long)avr->cycle, avr->data[24], avr->data[22]);
		int state = avr_run(avr);
		static int qstat = -1;
		static unsigned qs_qhead, qs_ring;
		static uint64_t qs_next, qs_hist[3][4][2];
		if (qstat < 0)
		  qstat = getenv("QSTAT") && sscanf(getenv("QSTAT"), "%x,%x", &qs_qhead, &qs_ring) == 2;
		if (qstat && fed < count && avr->cycle >= 300000 && avr->cycle >= qs_next)
		{
			qs_next = avr->cycle + 500;
			uint8_t g0 = avr->data[0x3E], qt = avr->data[0x4A], qh = avr->data[qs_qhead & 0xFFFF];
			int st = (g0 & 1) ? 0 : (g0 & 2) ? 1 : 2;                  /* RUN, DRAIN, IDLE */
			int q  = (uint8_t)(qh - qt) / 4;
			int qb = q == 0 ? 0 : q < 12 ? 1 : q < 40 ? 2 : 3;
			int rb = (uint8_t)(avr->data[qs_ring & 0xFFFF] - avr->data[(qs_ring & 0xFFFF) + 1]) != 0;
			qs_hist[st][qb][rb]++;
			/* where the CPU is while the transmitter starves */
			static FILE* qpc;
			if (st == 2 && q == 0 && rb && getenv("QSTAT_PC"))
			{
				if (!qpc) qpc = fopen(getenv("QSTAT_PC"), "w");
				fprintf(qpc, "%x\n", avr->pc);
			}
		}
		if (qstat && fed == count && avr->cycle >= end && (!rxgen || avr->cycle > 250000 + rx_nsent * frame + 100000))
		{
			static const char* sn[] = { "RUN", "DRAIN", "IDLE" };
			static const char* qn[] = { "q=0", "q1-11", "q12-39", "q40+" };
			uint64_t tot = 0;
			for (int a = 0; a < 3; a++) for (int b = 0; b < 4; b++) for (int c = 0; c < 2; c++) tot += qs_hist[a][b][c];
			printf("qstat (%% of samples; queue entries, tx ring empty / not):\n");
			for (int a = 0; a < 3; a++) for (int b = 0; b < 4; b++)
			  if (qs_hist[a][b][0] + qs_hist[a][b][1])
			    printf("  %-5s %-6s  ring empty %5.1f%%  ring data %5.1f%%\n", sn[a], qn[b],
			           100.0 * qs_hist[a][b][0] / tot, 100.0 * qs_hist[a][b][1] / tot);
			qstat = 0;
		}
		static uint64_t prof_to; if (!prof_to) prof_to = getenv("PROFILE_TO") ? strtoull(getenv("PROFILE_TO"), 0, 0) : ~0ULL;
		if (profiling && c0 >= prof_from && c0 < prof_to && pc / 2 < 16384)
		  prof[pc / 2] += (uint32_t)(avr->cycle - c0);
		if (state == cpu_Done || state == cpu_Crashed) { fprintf(stderr, "cpu stopped (%d)\n", state); return 2; }

		/* assert CTS once the firmware has set up its pull-up (an earlier
		 * external drive would be overridden by it in simavr) */
		static int cts_init;
		if (!cts_init && avr->cycle >= 150000 && !getenv("CTS_NEVER"))
		{
			cts_init = 1;
			set_cts(1);
		}
		/* hard limit: a stuck firmware fails instead of hanging */
		if (avr->cycle > 2000000000ULL)
		{
			printf("simulation did not finish\n");
			break;
		}
		if (cts_toggle && avr->cycle >= 300000)
		{
			static uint64_t next_cts = 300000;
			if (avr->cycle >= next_cts && ncts < 99999)
			{
				set_cts(!cts_on);
				next_cts = avr->cycle + (uint64_t)(frame * (cts_on ? 3 + rand() % 20 : 1 + rand() % 10)) + rand() % 1000;
			}
		}
		if (fed < count && avr->cycle >= next && !uart_xoff && fw_ready)
		{
			avr_raise_irq(uin, sent[fed++]);
			if (fed == 1) first_fed = avr->cycle;
			next += interval;
			if (next < avr->cycle) next = avr->cycle;   /* held off: don't burst to catch up */
			if (fed == count) end = avr->cycle + (uint64_t)(frame * 600) + 2000000;
		}
		if (count == 0 && !end) end = 250000 + (uint64_t)(rx_nsent * frame) + 2000000;
		if (fed == count && avr->cycle >= end && (!rxgen || avr->cycle > 250000 + rx_nsent * frame + 100000))
		  break;
	}
	nsent = count;
	if (profiling)
	{
		FILE* f = fopen(getenv("PROFILE"), "w");
		for (int i = 0; i < 16384; i++)
		  if (prof[i]) fprintf(f, "%x %u\n", i * 2, prof[i]);
		fclose(f);
	}
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

	if (nedges && nsent)
	  printf("tx throughput: %.1f%% of line rate\n",
	         100.0 * nsent * frame / (double)(edge_t[nedges - 1] - first_fed));

	/* 1. bytes back through USART1 (in RXGEN mode: the generated RX stream) */
	const uint8_t* want  = rxgen ? rx_sent : sent;
	int            nwant = rxgen ? rx_nsent : nsent;
	int bad = 0;
	for (int i = 0; i < nwant && i < nback; i++)
	  if (back[i] != want[i]) { if (bad < 5) printf("  rx byte %d: got %02x want %02x\n", i, back[i], want[i]); bad++; }
	printf("%s: sent %d, got %d, %d mismatches, error pin %d\n", rxgen ? "rx stream" : "loopback", nwant, nback, bad, err_pin);
	printf("main loop: longest iteration %llu cycles (at %llu); <500 %llu, <1000 %llu, <2000 %llu, <4000 %llu, <8000 %llu, more %llu\n",
	       (unsigned long long)loop_max, (unsigned long long)loop_max_at,
	       (unsigned long long)loop_hist[0], (unsigned long long)loop_hist[1], (unsigned long long)loop_hist[2],
	       (unsigned long long)loop_hist[3], (unsigned long long)loop_hist[4], (unsigned long long)loop_hist[5]);

	/* edges may be off by the encoder's rounding (1 timer tick) and the
	 * capture of frame starts (1 tick) */
	static const int presc[] = { 1, 8, 64, 256, 1024 };
	int cs = 0;
	while (cs < 4 && F_CPU / presc[cs] / baud * 12 > 65000) cs++;
	double tol = 2.0 * presc[cs] + 8;   /* + simavr step granularity (instructions, interrupt entry) */

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
		if (nstarts < 100000) starts[nstarts++] = edge_t[e];
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
		if (e < nedges && (double)edge_t[e] < s + (nb + 1 + 0.5 * stop) * bitc - tol)
		{ static int nearly; if (nearly++ < 3) { printf("  frame %d (start at cycle %.0f): next start bit %.1f cycles early; edges from its start:\n", ndec, s,
		                       s + (nb + 1 + 0.5 * stop) * bitc - edge_t[e]);
		             int e0 = e; while (e0 > 0 && (double)edge_t[e0] > s) e0--;
		             for (int j = e0; j <= e && j < nedges; j++)
		               printf("     %+9.2f bits -> %d\n", (edge_t[j] - s) / bitc, edge_v[j]); }
		  dbad++; }
	}
	/* inter-frame gaps (idle time between the end of one frame's stop bits
	 * and the next start bit) */
	{
		double fl = (nb + 1 + 0.5 * stop) * bitc, lost = 0, prev = -1;
		int ngaps = 0, nf = 0;
		for (int j = 0; j < nedges; j++)
		{
			if (edge_v[j] != 0) continue;
			/* a falling edge at least one frame after the previous start is a start bit */
			if (prev < 0 || edge_t[j] >= prev + fl - 6)
			{
				if (prev >= 0 && edge_t[j] > prev + fl + 6 && edge_t[j] < prev + 3 * fl) { lost += edge_t[j] - prev - fl; ngaps++; }
				prev = edge_t[j];
				nf++;
				if (getenv("GAPS") && nf == atoi(getenv("GAPS"))) { printf("frame %d starts at cycle %llu\n", nf, (unsigned long long)edge_t[j]); break; }
			}
		}
		printf("gaps: %d short idle gaps between back-to-back frames, %.0f cycles total (%.2f frames)\n", ngaps, lost, lost / fl);
	}
	/* CTS: a frame may only start while CTS is asserted, or within one
	 * frame (+ ISR latency) of its deassertion - the frame the transmitter
	 * had already committed to when CTS went away */
	int cts_bad = 0;
	if (cts_toggle)
	{
		double fl = (nb + 1 + 0.5 * stop) * (F_CPU / baud) + 300;
		int c = 0;
		for (int j = 0; j < nstarts; j++)
		{
			while (c + 1 < ncts && cts_t[c + 1] <= starts[j]) c++;
			if (ncts && cts_t[c] <= starts[j] && !cts_v[c] && starts[j] > cts_t[c] + fl)
			{
				if (cts_bad < 3) printf("  frame start at %llu while CTS deasserted since %llu\n",
				                       (unsigned long long)starts[j], (unsigned long long)cts_t[c]);
				cts_bad++;
			}
		}
		printf("cts: %d toggles, %d violations\n", ncts, cts_bad);
	}

	printf("waveform: %d frames, %d bad, worst edge deviation %.1f cycles (%.1f%% of a bit)\n",
	       ndec, dbad, worst, 100 * worst / bitc);

	int fail = cts_bad || bad || nback != nwant || err_pin || dbad || ndec != nsent || worst > tol;
	printf("%s\n", fail ? "FAIL" : "PASS");
	return fail;
}
