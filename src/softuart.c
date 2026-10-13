/*
 * Port 1: soft UART on Timer1, with all bit timing done by the timer
 * hardware and all framing work done in the main loop.
 *
 * Timer1 runs free (normal mode, TOP = 0xFFFF) with a prescaler picked per
 * baud rate so that a whole frame fits in 16 bits.
 *
 * TX: the main loop encodes each byte into the line transitions ("edges")
 *     it needs (softuart_codec.c) and queues them. Output compare A drives
 *     the OC1A pin in hardware - on a match the pin is set or cleared as
 *     programmed in COM1A - so edges are exact regardless of interrupt
 *     latency. The compare ISR only programs the next one: OCR1A += delta
 *     and the new COM1A mode. Its deadline is that next edge, at least one
 *     bit time away.
 *
 * RX: input capture timestamps edges on ICP1. The capture ISR stores the
 *     16-bit timestamp and flips the edge polarity; its deadline is also
 *     the next edge. Because the polarity is flipped after every capture,
 *     captured edges always alternate falling/rising, so the level after
 *     each one follows from the current ICES1 setting and is not stored.
 *     The main loop extends timestamps to 32 bits and turns the edge stream
 *     into bytes by sampling each bit in its middle.
 *
 * Both ISRs are hand written: they are what limits the usable baud rate.
 * At 16 MHz and 115200 baud a bit is 139 cycles; the capture ISR flips the
 * polarity 23 cycles after entry and takes 55 in total, the compare ISR
 * reprograms OCR1A 43 cycles after entry and takes 63. The USART ISRs
 * re-enable interrupts within ~20 cycles (see hwuart.c), so the worst case
 * from a compare match to OCR1A being updated is ~20 + 55 + 43 = 118 cycles.
 *
 * Main loop requirement: softuart_task() must run at least once per Timer1
 * wrap (65536 ticks: 4 ms at 115200, more at lower rates) to keep the
 * 32-bit time extension right, and often enough to keep the edge ring
 * (128 edges, >11 frames) from overflowing.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "softuart.h"
#include "softuart_codec.h"
#include "config.h"

#define RX_PORT    PORTD
#define RX_PIN     PIND
#define RX_BIT     PD4
#define TX_PORT    PORTB
#define TX_DDR     DDRB
#define TX_BIT     PB5

/* TCCR1A values: OC1A set (line high) / cleared (line low) on match */
#define COM_SET    0xC0       /* COM1A1 | COM1A0 */
#define COM_CLEAR  0x80       /* COM1A1 */
#define CTL_START  0x01       /* queue flag: first edge of a frame (masked off before TCCR1A) */

/* how far ahead of "now" a transmission restarting from idle is scheduled */
#define TX_LEAD    32

/* GPIOR0 bits: transmitter state (neither = idle, compare interrupt off) */
#define TXS_RUN    0          /* OCR1A holds the next queued edge */
#define TXS_DRAIN  1          /* OCR1A holds the end of the last frame's stop bits */

/* Edge ring (RX) and edge queue (TX) are 256-byte aligned rings of 2 and
 * 4 byte entries indexed by a byte offset, so the ISRs index with a single
 * register and wrap for free. The ISR-owned index of each lives in a GPIOR. */
#define E_HEAD     GPIOR2     /* written by the capture ISR */
#define Q_TAIL     GPIOR1     /* written by the compare ISR */

static struct
{
	uint8_t e[256];           /* {ICR1L, ICR1H} */
	uint8_t q[256];           /* {delta lo, delta hi, TCCR1A | CTL_START, unused} */
} bufs __attribute__((aligned(256), section(".noinit")));

volatile uint8_t  su_e_tail;        /* edge ring tail, main loop */
volatile uint8_t  su_e_overrun;     /* set by the capture ISR when the ring is full */
volatile uint8_t  su_q_head;        /* edge queue head, main loop */
volatile uint16_t su_tx_tail_ticks; /* stop bit time left after the last queued frame's last edge */

#define Q_ENTRIES_FREE() ((uint8_t)(255 - (uint8_t)(su_q_head - Q_TAIL)) / 4)

/* ---- main loop state ---- */

static ring_t           tx_ring, rx_ring;
static volatile uint8_t errors;
static su_cfg_t         cfg;
static su_enc_t         enc;
static su_dec_t         dec;
static uint16_t         time_hi;    /* upper half of the 32-bit time */
static uint16_t         time_lo;    /* TCNT1 at the last timer_now() */

/* ------------------------------------------------------------------------ */

ISR(TIMER1_CAPT_vect, ISR_NAKED)
{
	asm volatile(
		"push r24"                  "\n\t"
		"in   r24, __SREG__"        "\n\t"
		"push r24"                  "\n\t"
		"push r30"                  "\n\t"
		"push r31"                  "\n\t"
		/* wait for the opposite edge next; changing ICES1 can set ICF1 */
		"lds  r24, %[tccr1b]"       "\n\t"
		"ldi  r30, %[ices]"         "\n\t"
		"eor  r30, r24"             "\n\t"
		"sts  %[tccr1b], r30"       "\n\t"
		"ldi  r30, %[icf]"          "\n\t"
		"out  %[tifr1], r30"        "\n\t"
		/* store the timestamp at the head */
		"in   r30, %[head]"         "\n\t"
		"ldi  r31, hi8(%[ebuf])"    "\n\t"
		"lds  r24, %[icr1l]"        "\n\t"
		"st   Z+, r24"              "\n\t"
		"lds  r24, %[icr1h]"        "\n\t"
		"st   Z+, r24"              "\n\t"
		/* publish it, unless that would make head == tail */
		"lds  r24, %[tail]"         "\n\t"
		"cpse r30, r24"             "\n\t"
		"rjmp 1f"                   "\n\t"
		"sts  %[ovr], r31"          "\n\t"   /* r31 is never 0 */
		"rjmp 2f"                   "\n\t"
	"1:"	"out  %[head], r30"         "\n\t"
	"2:"	"pop  r31"                  "\n\t"
		"pop  r30"                  "\n\t"
		"pop  r24"                  "\n\t"
		"out  __SREG__, r24"        "\n\t"
		"pop  r24"                  "\n\t"
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

ISR(TIMER1_COMPA_vect, ISR_NAKED)
{
	asm volatile(
		"push r24"                  "\n\t"
		"in   r24, __SREG__"        "\n\t"
		"push r24"                  "\n\t"
		"push r25"                  "\n\t"
		"push r30"                  "\n\t"
		"push r31"                  "\n\t"
		"in   r30, %[qtail]"        "\n\t"
		"lds  r24, %[qhead]"        "\n\t"
		"sbis %[state], %[run]"     "\n\t"
		"rjmp 3f"                   "\n\t"
		"cp   r30, r24"             "\n\t"
		"breq 2f"                   "\n\t"

		/* RUN: program the next queued edge */
		"ldi  r31, hi8(%[qbuf])"    "\n\t"
		"ld   r25, Z+"              "\n\t"   /* delta lo */
		"lds  r24, %[ocrl]"         "\n\t"
		"add  r24, r25"             "\n\t"
		"ld   r25, Z+"              "\n\t"   /* delta hi (ld keeps the carry) */
		"ld   r30, Z"               "\n\t"   /* control */
		"lds  r31, %[ocrh]"         "\n\t"
		"adc  r31, r25"             "\n\t"
		"sts  %[ocrh], r31"         "\n\t"
		"sts  %[ocrl], r24"         "\n\t"
		"andi r30, %[commask]"      "\n\t"
		"sts  %[tccr1a], r30"       "\n\t"
		"in   r24, %[qtail]"        "\n\t"
		"subi r24, -4"              "\n\t"
		"out  %[qtail], r24"        "\n\t"
	"1:"	"pop  r31"                  "\n\t"
		"pop  r30"                  "\n\t"
		"pop  r25"                  "\n\t"
		"pop  r24"                  "\n\t"
		"out  __SREG__, r24"        "\n\t"
		"pop  r24"                  "\n\t"
		"reti"                      "\n\t"

		/* RUN, queue empty: time the end of the stop bits */
	"2:"	"lds  r24, %[ocrl]"         "\n\t"
		"lds  r25, %[ttail]"        "\n\t"
		"add  r24, r25"             "\n\t"
		"lds  r25, %[ttail]+1"      "\n\t"
		"lds  r31, %[ocrh]"         "\n\t"
		"adc  r31, r25"             "\n\t"
		"sts  %[ocrh], r31"         "\n\t"
		"sts  %[ocrl], r24"         "\n\t"
		"ldi  r24, %[comset]"       "\n\t"
		"sts  %[tccr1a], r24"       "\n\t"
		"cbi  %[state], %[run]"     "\n\t"
		"sbi  %[state], %[drain]"   "\n\t"
		"rjmp 1b"                   "\n\t"

		/* DRAIN: the stop bits are done */
	"3:"	"cp   r30, r24"             "\n\t"
		"breq 4f"                   "\n\t"
		/* a frame was queued meanwhile: start it now (skipping its
		 * start edge's delta, which assumed back to back frames) */
		"subi r30, -4"              "\n\t"
		"out  %[qtail], r30"        "\n\t"
		"lds  r24, %[tcntl]"        "\n\t"
		"lds  r25, %[tcnth]"        "\n\t"
		"adiw r24, %[lead]"         "\n\t"
		"sts  %[ocrh], r25"         "\n\t"
		"sts  %[ocrl], r24"         "\n\t"
		"ldi  r24, %[comclr]"       "\n\t"
		"sts  %[tccr1a], r24"       "\n\t"
		"cbi  %[state], %[drain]"   "\n\t"
		"sbi  %[state], %[run]"     "\n\t"
		"rjmp 1b"                   "\n\t"
		/* nothing to send: go idle */
	"4:"	"lds  r24, %[timsk1]"       "\n\t"
		"andi r24, %[nocie]"        "\n\t"
		"sts  %[timsk1], r24"       "\n\t"
		"cbi  %[state], %[drain]"   "\n\t"
		"rjmp 1b"                   "\n\t"
		::
		[ocrl]    "n" (_SFR_MEM_ADDR(OCR1AL)),
		[ocrh]    "n" (_SFR_MEM_ADDR(OCR1AH)),
		[tcntl]   "n" (_SFR_MEM_ADDR(TCNT1L)),
		[tcnth]   "n" (_SFR_MEM_ADDR(TCNT1H)),
		[tccr1a]  "n" (_SFR_MEM_ADDR(TCCR1A)),
		[timsk1]  "n" (_SFR_MEM_ADDR(TIMSK1)),
		[qtail]   "I" (_SFR_IO_ADDR(Q_TAIL)),
		[state]   "I" (_SFR_IO_ADDR(GPIOR0)),
		[run]     "I" (TXS_RUN),
		[drain]   "I" (TXS_DRAIN),
		[commask] "M" (~CTL_START & 0xFF),
		[comset]  "M" (COM_SET),
		[comclr]  "M" (COM_CLEAR),
		[nocie]   "M" (~_BV(OCIE1A) & 0xFF),
		[lead]    "I" (TX_LEAD),
		[qbuf]    "i" (bufs.q),
		[qhead]   "i" (&su_q_head),
		[ttail]   "i" (&su_tx_tail_ticks)
	);
}

/* ------------------------------------------------------------------------ */

static void rx_byte(uint8_t data, uint8_t flags)
{
	errors |= flags;
	if (flags & SU_RX_BREAK)
	  return;

	if (!ring_free(&rx_ring))
	{
		errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
		return;
	}
	ring_put(&rx_ring, data);
}

static void rx_task(void)
{
	uint16_t tcnt;
	uint8_t  head, tccr1b;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		tcnt = TCNT1;
		/* An edge captured while interrupts were masked is not in the ring
		 * yet: take "now" as the time of that edge instead, so no sample
		 * beyond it is taken with the old line level. */
		if (TIFR1 & _BV(ICF1))
		  tcnt = ICR1;
		head   = E_HEAD;
		tccr1b = TCCR1B;
	}

	/* extend to 32 bits (we run at least once per wrap) */
	if (tcnt < time_lo)
	  time_hi++;
	time_lo = tcnt;
	uint32_t now = ((uint32_t)time_hi << 16) | tcnt;

	/* Captures alternate in polarity, and ICES1 is set up for the one after
	 * the newest: if it waits for a rising edge, the newest was falling.
	 * Work back from there to the level after the oldest unprocessed one. */
	uint8_t t     = su_e_tail;
	uint8_t count = (uint8_t)(head - t) / 2;
	uint8_t level = (tccr1b & _BV(ICES1)) ? 0 : 1;
	if (!(count & 1))
	  level ^= 1;

	while (t != head)
	{
		uint16_t ts = bufs.e[t] | ((uint16_t)bufs.e[(uint8_t)(t + 1)] << 8);
		su_dec_edge(&dec, &cfg, now - (uint16_t)(tcnt - ts), level, rx_byte);
		level ^= 1;
		t += 2;
	}
	su_e_tail = t;

	su_dec_time(&dec, &cfg, now, rx_byte);

	if (su_e_overrun)
	{
		su_e_overrun = 0;
		errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
	}
}

/* Encode bytes from the tx ring into the edge queue while there is room
 * for a worst case frame, then make sure the transmitter is running. */
static void softuart_tx_start(void)
{
	while (!ring_empty(&tx_ring) && Q_ENTRIES_FREE() >= SU_MAX_EDGES)
	{
		su_edge_t e[SU_MAX_EDGES];
		uint8_t   n = su_encode(&cfg, &enc, ring_get(&tx_ring), e);
		uint8_t   h = su_q_head;

		for (uint8_t j = 0; j < n; j++, h += 4)
		{
			bufs.q[h]     = (uint8_t)e[j].delta;
			bufs.q[h + 1] = (uint8_t)(e[j].delta >> 8);
			bufs.q[h + 2] = (e[j].level ? COM_SET : COM_CLEAR) | (e[j].start ? CTL_START : 0);
		}

		/* publish the whole frame at once so the ISR never sees half of it */
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			su_q_head        = h;
			su_tx_tail_ticks = enc.tail;
		}
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		uint8_t t = Q_TAIL;
		if (!(GPIOR0 & (_BV(TXS_RUN) | _BV(TXS_DRAIN))) && t != su_q_head)
		{
			/* the first queued edge is a start bit; begin right away */
			Q_TAIL  = t + 4;
			OCR1A   = TCNT1 + TX_LEAD;
			TCCR1A  = COM_CLEAR;
			TIFR1   = _BV(OCF1A);
			TIMSK1 |= _BV(OCIE1A);
			GPIOR0 |= _BV(TXS_RUN);
		}
	}
}

static bool softuart_tx_idle(void)
{
	return !(GPIOR0 & (_BV(TXS_RUN) | _BV(TXS_DRAIN))) && Q_TAIL == su_q_head && ring_empty(&tx_ring);
}

static void softuart_set_break(bool on)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TCCR1A = on ? COM_CLEAR : COM_SET;
		TCCR1C = _BV(FOC1A);
	}
	su_enc_reset(&enc);
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

	/* smallest prescaler for which the longest frame fits the timer */
	static const uint16_t prescalers[] = { 1, 8, 64, 256, 1024 };
	uint8_t cs = 0;
	while (cs < 4 && F_CPU / prescalers[cs] / baud * SU_MAX_FRAME_BITS > 65000)
	  cs++;

	su_cfg_init(&cfg, F_CPU / prescalers[cs], baud, le->DataBits, le->ParityType, le->CharFormat);

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		uint8_t line = (RX_PIN >> RX_BIT) & 1;

		TIMSK1 &= ~_BV(OCIE1A);
		GPIOR0 &= ~(_BV(TXS_RUN) | _BV(TXS_DRAIN));
		Q_TAIL  = su_q_head;
		ring_flush(&tx_ring);
		su_enc_reset(&enc);

		/* capture the next edge: falling if the line is high now */
		TCCR1B = _BV(ICNC1) | (line ? 0 : _BV(ICES1)) | (cs + 1);
		TIFR1  = _BV(ICF1);
		su_e_tail = E_HEAD;
		su_dec_reset(&dec, line);
		time_lo = TCNT1;
	}
}

static void softuart_task(void)
{
	rx_task();
}

void softuart_init(void)
{
	TX_PORT |= _BV(TX_BIT);
	TX_DDR  |= _BV(TX_BIT);
	RX_PORT |= _BV(RX_BIT);          /* pull-up on RX */

	GPIOR0 = 0;
	TCCR1A = COM_SET;
	TCCR1C = _BV(FOC1A);             /* OC1A high: line idle */
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
};
