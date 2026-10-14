/*
 * Port 1: soft UART, with all bit timing done by timer hardware and all
 * framing work done in the main loop.
 *
 * TX: Timer3 runs in CTC mode with a period of one bit, and its compare ISR
 *     fires once per bit. The main loop turns each byte into a 16-bit frame
 *     word (start bit in bit 0, then data, parity and stop bits) and queues
 *     it. The ISR shifts the word out a bit at a time, programming OC3A
 *     (PC6) to be set or cleared on the next match, so the line switches
 *     exactly on the bit grid whatever the interrupt latency; the ISR's
 *     deadline is that next match, one bit time away. Bit times are whole
 *     timer ticks: at most 0.5 tick off per bit (0.08% at 115200), and
 *     1.5 stop bits are sent as 2.
 *
 * RX: Timer1 runs free (normal mode) with a prescaler picked per baud rate
 *     so that a whole frame fits in 16 bits. Input capture timestamps edges
 *     on ICP1 (PD4). The capture ISR stores the 16-bit timestamp and flips
 *     the edge polarity; its deadline is the next edge. Because the
 *     polarity is flipped after every capture, captured edges always
 *     alternate falling/rising, so the level after each one follows from
 *     the current ICES1 setting and is not stored. The main loop turns the
 *     edge stream into bytes by sampling each bit in its middle
 *     (softuart_dec.S).
 *
 * CTS flow control (P1_HW_FLOW) is checked by the TX ISR when it is about
 * to start a frame, so a frame in progress always completes.
 *
 * Both ISRs are hand written and keep their state in registers reserved
 * for them (the Makefile builds everything with -ffixed-r2 .. r7), so they
 * save next to nothing. Counting from the interrupt request (4 cycle
 * response + jmp): the TX ISR has written TCCR3A after 12 cycles and takes
 * 21 per bit, ~52 when it fetches the next frame word; the capture ISR has
 * flipped the polarity after 18 cycles and takes 45. The USART ISRs
 * re-enable interrupts after ~17 cycles (see hwuart.c). At 115200 a bit is
 * 139 cycles; the worst cases are ~17 + 45 + 12 = 74 cycles from a compare
 * match to TCCR3A written and ~17 + 52 + 18 = 87 from an edge to its
 * capture being re-armed.
 *
 * Main loop requirement: softuart_task() must run at least once per Timer1
 * wrap minus a frame (~3.5 ms at 115200, more at lower rates) because the
 * decoder works with 16-bit time, and often enough to keep the edge ring
 * (127 edges, >11 frames) from overflowing.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "softuart.h"
#include "softuart_codec.h"
#include "config.h"
#include "profiler.h"
#include "gpio.h"

#if P1_HW_FLOW == FLOW_ON
#define CTS_BLOCKED() GPIO_READ(P1_CTS_PIN)    /* deasserted = high */
#elif P1_HW_FLOW == FLOW_AUTO
#define CTS_BLOCKED() ((GPIOR0 & _BV(TXS_CTSARM)) && GPIO_READ(P1_CTS_PIN))
#else
#define CTS_BLOCKED() 0
#endif

#ifndef SU_REGS_RESERVED
#error the soft UART ISRs need r2-r7 reserved (-ffixed-r2 ... -ffixed-r7, see the Makefile)
#endif

#define RX_PORT    PORTD
#define RX_PIN     PIND
#define RX_BIT     PD4
#define TX_PORT    PORTC
#define TX_DDR     DDRC
#define TX_BIT     PC6

/* TCCR3A values: OC3A set (line high) / cleared (line low) on match */
#define COM_SET    (_BV(COM3A1) | _BV(COM3A0))
#define COM_CLEAR  _BV(COM3A1)

/* debugging hook for the simulator: marks decoder runs */
#ifndef SU_TRACE_DECODE
#define SU_TRACE_DECODE()
#endif

/* GPIOR0 bits */
#define TXS_CTSARM 2          /* FLOW_AUTO: CTS is being obeyed */

/* The edge ring (RX) and the frame word queue (TX) are 256-byte aligned
 * rings of 2-byte entries indexed by a byte offset, so the ISRs index with
 * a single register and wrap for free. The ISR-owned index of each lives in
 * a GPIOR. */
#define E_HEAD     GPIOR2     /* written by the capture ISR */
#define TXW_TAIL   GPIOR1     /* written by the TX ISR */

static struct
{
	uint8_t e[256];           /* {ICR1L, ICR1H} */
	uint8_t q[256];           /* frame words, little endian */
} bufs __attribute__((aligned(256), section(".noinit")));

volatile uint8_t  su_e_tail;        /* edge ring tail, main loop */
volatile uint8_t  su_e_overrun;     /* set by the capture ISR when the ring is full */

/* ---- main loop state ---- */

static ring_t           tx_ring, rx_ring;
static volatile uint8_t errors;
static su_cfg_t         cfg;
static su_dec_t         dec;

/* Registers reserved for the ISRs (the Makefile builds everything with
 * -ffixed for them):
 *   r3:r2  TX: frame word being sent, next bit in bit 0
 *   r4     TX: bits of it left to program
 *   r5     TX: TCCR3A value, COM_CLEAR with bit 6 set from the frame word
 *   r6     SREG, saved by either ISR (neither can interrupt the other)
 *   r7     capture ISR: data register */

volatile uint8_t su_txw_head;       /* frame word queue head, main loop */
static uint8_t   su_tx_fbits;       /* bits per frame, start bit included */

/* the ISR's state for an idle line: one more 1 bit, then fetch a word */
static inline void tx_isr_idle(void)
{
	asm volatile(
		"ldi  r24, 0xFF"    "\n\t"
		"mov  r2, r24"      "\n\t"
		"ldi  r24, 1"       "\n\t"
		"mov  r4, r24"      "\n\t"
		"ldi  r24, %0"      "\n\t"
		"mov  r5, r24"      "\n\t"
		:: "M" (COM_CLEAR) : "r24");
}

/* ------------------------------------------------------------------------ */

ISR(TIMER1_CAPT_vect, ISR_NAKED)
{
	asm volatile(
		"in   r6, __SREG__"         "\n\t"
		"push r30"                  "\n\t"
		"push r31"                  "\n\t"
		/* wait for the opposite edge next; changing ICES1 can set ICF1 */
		"lds  r7, %[tccr1b]"        "\n\t"
		"ldi  r30, %[ices]"         "\n\t"
		"eor  r30, r7"              "\n\t"
		"sts  %[tccr1b], r30"       "\n\t"
		"ldi  r30, %[icf]"          "\n\t"
		"out  %[tifr1], r30"        "\n\t"
		/* store the timestamp at the head */
		"in   r30, %[head]"         "\n\t"
		"ldi  r31, hi8(%[ebuf])"    "\n\t"
		"lds  r7, %[icr1l]"         "\n\t"
		"st   Z+, r7"               "\n\t"
		"lds  r7, %[icr1h]"         "\n\t"
		"st   Z+, r7"               "\n\t"
		/* publish it, unless that would make head == tail */
		"lds  r7, %[tail]"          "\n\t"
		"cpse r30, r7"              "\n\t"
		"rjmp 1f"                   "\n\t"
		"sts  %[ovr], r31"          "\n\t"   /* r31 is never 0 */
		"rjmp 2f"                   "\n\t"
	"1:"	"out  %[head], r30"         "\n\t"
	"2:"	"pop  r31"                  "\n\t"
		"pop  r30"                  "\n\t"
		"out  __SREG__, r6"         "\n\t"
		"reti"                      "\n\t"
		::
		[tccr1b] "n" (_SFR_MEM_ADDR(TCCR1B)),
		[icr1l]  "n" (_SFR_MEM_ADDR(ICR1L)),
		[icr1h]  "n" (_SFR_MEM_ADDR(ICR1H)),
		[tifr1]  "I" (_SFR_IO_ADDR(TIFR1)),
		[head]   "I" (_SFR_IO_ADDR(E_HEAD)),
		[ices]   "M" (_BV(ICES1)),
		[icf]    "M" (_BV(ICF1)),
		[ebuf]   "i" (bufs.e),
		[tail]   "i" (&su_e_tail),
		[ovr]    "i" (&su_e_overrun)
	);
}


ISR(TIMER3_COMPA_vect, ISR_NAKED)
{
	asm volatile(
		"in   r6, __SREG__"         "\n\t"
		"bst  r2, 0"                "\n\t"
		"bld  r5, 6"                "\n\t"   /* COM_CLEAR or COM_SET */
		"sts  %[tccr3a], r5"        "\n\t"
		"lsr  r3"                   "\n\t"
		"ror  r2"                   "\n\t"
		"dec  r4"                   "\n\t"
		"breq 2f"                   "\n\t"
	"1:"	"out  __SREG__, r6"         "\n\t"
		"reti"                      "\n\t"

		/* that was the frame's last bit (a stop bit): fetch the next word */
	"2:"	"push r30"                  "\n\t"
		"push r31"                  "\n\t"
		"in   r30, %[tail]"         "\n\t"
		"lds  r31, %[head]"         "\n\t"
		"cp   r30, r31"             "\n\t"
		"breq 4f"                   "\n\t"
#if P1_HW_FLOW
		/* only start a frame while CTS is asserted (low) */
#if P1_HW_FLOW == FLOW_AUTO
		"sbis %[state], %[ctsarm]"  "\n\t"   /* not armed: CTS doesn't count */
		"rjmp 5f"                   "\n\t"
#endif
		"sbic %[ctspin], %[ctsbit]" "\n\t"
		"rjmp 4f"                   "\n\t"
	"5:"
#endif
		"ldi  r31, hi8(%[qbuf])"    "\n\t"
		"ld   r2, Z+"               "\n\t"
		"ld   r3, Z+"               "\n\t"
		"out  %[tail], r30"         "\n\t"
		"lds  r4, %[fbits]"         "\n\t"
	"3:"	"pop  r31"                  "\n\t"
		"pop  r30"                  "\n\t"
		"rjmp 1b"                   "\n\t"

		/* Nothing to start (queue empty or CTS holding it off): stop
		 * right away rather than polling every bit, in the state "one
		 * more 1 bit, then fetch". The last stop bit is programmed, so a
		 * restart by the main loop adds at least a bit of idle line and
		 * never cuts the stop bit short. */
	"4:"	"ldi  r30, 1"               "\n\t"
		"mov  r2, r30"              "\n\t"
		"mov  r4, r30"              "\n\t"
		"lds  r30, %[timsk3]"       "\n\t"
		"andi r30, ~%[ocie]"        "\n\t"
		"sts  %[timsk3], r30"       "\n\t"
		"rjmp 3b"                   "\n\t"
		::
		[tccr3a]   "n" (_SFR_MEM_ADDR(TCCR3A)),
		[timsk3]   "n" (_SFR_MEM_ADDR(TIMSK3)),
		[ocie]     "M" (_BV(OCIE3A)),
		[tail]     "I" (_SFR_IO_ADDR(TXW_TAIL)),
		[head]     "i" (&su_txw_head),
		[fbits]    "i" (&su_tx_fbits),
		[state]    "I" (_SFR_IO_ADDR(GPIOR0)),
		[ctsarm]   "I" (TXS_CTSARM),
		[qbuf]     "i" (bufs.q),
		[ctspin]   "I" (_SFR_IO_ADDR(GPIO_PINREG(P1_CTS_PIN))),
		[ctsbit]   "I" (GPIO_BIT(P1_CTS_PIN))
	);
}


/* ------------------------------------------------------------------------ */

#if PROFILER
/* event counters for the profiler report (see profiler.h) */
volatile uint16_t su_stats[SU_STATS];
#define SU_STAT(i) do { if (su_stats[i] != 0xFFFF) su_stats[i]++; } while (0)
#else
#define SU_STAT(i) do { } while (0)
#endif

void su_rx_frame(uint8_t data, uint8_t flags)
{
	errors |= flags;
	if (flags & SU_RX_BREAK)
	  return;

	if (!ring_free(&rx_ring))
	{
		SU_STAT(7);
		errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
		return;
	}
	ring_put(&rx_ring, data);
}

static void rx_task(void)
{
	uint16_t now;
	uint8_t  head, tccr1b;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		now = TCNT1;
		/* An edge captured while interrupts were masked is not in the ring
		 * yet: take "now" as the time of that edge instead, so no sample
		 * beyond it is taken with the old line level. */
		if (TIFR1 & _BV(ICF1))
		  now = ICR1;
		head   = E_HEAD;
		tccr1b = TCCR1B;
	}

	uint8_t t = su_e_tail;

	/* Nothing new, and the frame in progress (if any) isn't over yet: its
	 * pending samples can wait for the next edge, only the last one must be
	 * taken without one (a frame ending in 1s has no closing edge). */
	if (t == head && (!dec.in_frame || (uint16_t)(now - dec.start) <= cfg.last_sample))
	  goto check_overrun;

	/* Captures alternate in polarity, and ICES1 is set up for the one after
	 * the newest: if it waits for a rising edge, the newest was falling.
	 * Work back from there to the level after the oldest unprocessed one. */
	uint8_t level = (tccr1b & _BV(ICES1)) ? 0 : 1;
	if (!(((uint8_t)(head - t) / 2) & 1))
	  level ^= 1;

	SU_TRACE_DECODE();
	su_dec_run(&dec, &cfg, bufs.e, t, head, level, now);
	su_e_tail = head;

check_overrun:
	if (su_e_overrun)
	{
		su_e_overrun = 0;
		SU_STAT(6);
		errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
	}
}

/* Queue frame words for bytes from the tx ring while there is room, then
 * make sure the transmitter is running.
 *
 * At most TX_ENCODE_BATCH frames are queued per call, which keeps main loop
 * iterations short so the receive side (which, unlike TX, cannot be held
 * off) is serviced often. When the CPU runs short at high rates, TX then
 * slows down by leaving gaps between frames instead of RX losing data. */
#ifndef TX_ENCODE_BATCH
#define TX_ENCODE_BATCH 4
#endif

/* Receive has priority: while received edges are piling up (the decoder is
 * falling behind), don't queue more TX frames - fewer frames on the wire
 * also means fewer TX interrupts. */
#ifndef RX_BACKLOG_MAX
#define RX_BACKLOG_MAX  20      /* edges; RX loses data in the simulator from ~36 */
#endif

/* Frames queued ahead of the transmitter: enough to ride out a long main
 * loop iteration, and no more, because the gate above only holds off new
 * frames. Queued ones still go out, and while received edges pile up the
 * edge ring lasts ~12 frames (0x55 in 8N1, an edge every bit). With the
 * whole queue (127 frames) a TX burst queued during an RX pause overran it
 * once RX resumed. */
#ifndef TXQ_MAX
#define TXQ_MAX 8
#endif

/* The frame word: start bit (0) in bit 0, then the data, parity and stop
 * bits (ones from there up) */
static uint16_t tx_word(uint8_t data)
{
	data &= cfg.data_mask;
	uint8_t p = data ^ (data >> 4);
	p ^= p >> 2;
	p ^= p >> 1;
	uint8_t lut = cfg.par_lut;
	if (p & 1)
	  lut >>= 1;
	uint16_t d = data | cfg.enc_or;
	if (lut & 1)
	  d |= cfg.enc_par;
	return d << 1;
}

static void softuart_tx_start(void)
{
	/* checked once per call: a batch adds at most a few frames' worth of
	 * edges to the RX backlog meanwhile, far from filling the edge ring */
	uint8_t n = ((uint8_t)(E_HEAD - su_e_tail) <= 2 * RX_BACKLOG_MAX) ? TX_ENCODE_BATCH : 0;

	SU_STAT(0);
	if (!n && !ring_empty(&tx_ring))
	  SU_STAT(1);

	for (uint8_t i = 0; i < n; i++)
	{
		uint8_t h = su_txw_head;
		if (ring_empty(&tx_ring))            { SU_STAT(3); break; }
		if ((uint8_t)(h - TXW_TAIL) >= 2 * TXQ_MAX) { SU_STAT(4); break; }
		if (PROFILER && i == n - 1 && ring_count(&tx_ring) > 1)
		  SU_STAT(2);
		uint16_t w = tx_word(ring_get(&tx_ring));
		bufs.q[h]     = w;
		bufs.q[h + 1] = w >> 8;
		asm volatile("" ::: "memory");      /* the word before the head */
		su_txw_head   = h + 2;
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		/* (the ISR stops itself when it finds nothing to start) */
		if (!(TIMSK3 & _BV(OCIE3A)) && TXW_TAIL != su_txw_head && !CTS_BLOCKED())
		{
			SU_STAT(5);
			TIMSK3 |= _BV(OCIE3A);
		}
	}
}

static bool softuart_tx_idle(void)
{
	if (TIMSK3 & _BV(OCIE3A))
	  return false;
	return (TXW_TAIL == su_txw_head && ring_empty(&tx_ring)) || CTS_BLOCKED();
}

static void softuart_set_break(bool on)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TCCR3A = on ? COM_CLEAR : COM_SET;
		TCCR3C = _BV(FOC3A);
	}
}

static void softuart_configure(const CDC_LineEncoding_t* le)
{
	uint32_t baud = le->BaudRateBPS;
	if (baud == 0)
	  baud = 9600;
	if (baud < SOFTUART_MIN_BAUD)
	  baud = SOFTUART_MIN_BAUD;
	if (baud > SOFTUART_MAX_BAUD)
	  baud = SOFTUART_MAX_BAUD;

	/* Timer1 (RX): smallest prescaler for which the longest frame fits */
	static const uint16_t prescalers[] = { 1, 8, 64, 256, 1024 };
	uint8_t cs = 0;
	while (cs < 4 && F_CPU / prescalers[cs] / baud * SU_MAX_FRAME_BITS > 65000)
	  cs++;

	su_cfg_init(&cfg, F_CPU / prescalers[cs], baud, le->DataBits, le->ParityType, le->CharFormat);

	/* Timer3 (TX): one bit per period, the smallest prescaler that fits */
	uint8_t cs3 = 0;
	while (cs3 < 4 && F_CPU / prescalers[cs3] / baud > 65535)
	  cs3++;
	uint16_t period = (F_CPU / prescalers[cs3] + baud / 2) / baud;
	/* 1.5 stop bits are sent as 2 */
	uint8_t fbits = 1 + cfg.data_bits + (cfg.parity != SU_PARITY_NONE) + (le->CharFormat ? 2 : 1);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		uint8_t line = (RX_PIN >> RX_BIT) & 1;

		TIMSK3 &= ~_BV(OCIE3A);
		TXW_TAIL    = su_txw_head;
		su_tx_fbits = fbits;
		tx_isr_idle();
		TCCR3B = 0;
		TCNT3  = 0;
		OCR3A  = period - 1;
		TCCR3A = COM_SET;
		TCCR3C = _BV(FOC3A);
		TCCR3B = _BV(WGM32) | (cs3 + 1);
		ring_flush(&tx_ring);

		/* capture the next edge: falling if the line is high now */
		TCCR1B = _BV(ICNC1) | (line ? 0 : _BV(ICES1)) | (cs + 1);
		TIFR1  = _BV(ICF1);
		su_e_tail = E_HEAD;
		su_dec_reset(&dec, line);
	}
}

static void softuart_task(void)
{
	rx_task();
}

static void softuart_cts_arm(bool on)
{
	if (on)
	  GPIOR0 |= _BV(TXS_CTSARM);     /* sbi/cbi: atomic */
	else
	  GPIOR0 &= ~_BV(TXS_CTSARM);
}

void softuart_init(void)
{
	TX_PORT |= _BV(TX_BIT);
	TX_DDR  |= _BV(TX_BIT);
	RX_PORT |= _BV(RX_BIT);          /* pull-up on RX */

	GPIOR0 = 0;
	TCCR1A = 0;
	tx_isr_idle();
	TCCR3A = COM_SET;
	TCCR3C = _BV(FOC3A);             /* OC3A high: line idle */
	TIMSK1 = _BV(ICIE1);

	static const CDC_LineEncoding_t def =
		{ .BaudRateBPS = 9600, .CharFormat = CDC_LINEENCODING_OneStopBit,
		  .ParityType = CDC_PARITY_None, .DataBits = 8 };
	softuart_configure(&def);
}

const uart_port_t softuart_port =
{
	.tx        = &tx_ring,
	.rx        = &rx_ring,
	.errors    = &errors,
	.configure = softuart_configure,
	.task      = softuart_task,
	.tx_start  = softuart_tx_start,
	.tx_idle   = softuart_tx_idle,
	.set_break = softuart_set_break,
	.cts_arm   = softuart_cts_arm,
};
