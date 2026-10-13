/*
 * Port 1: soft UART on Timer1, with all bit timing done by the timer
 * hardware and all framing work done in the main loop.
 *
 * Timer1 runs free (normal mode, TOP = 0xFFFF) with a prescaler picked per
 * baud rate so that a whole frame fits in 16 bits.
 *
 * TX (SU_TX_MODE 0): the main loop encodes each byte into the line transitions ("edges")
 *     it needs (softuart_codec.c) and queues them. Output compare A drives
 *     the OC1A pin in hardware - on a match the pin is set or cleared as
 *     programmed in COM1A - so edges are exact regardless of interrupt
 *     latency. The compare ISR only programs the next one: OCR1A += delta
 *     and the new COM1A mode. Its deadline is that next edge, at least one
 *     bit time away.
 *
 * TX (SU_TX_MODE 3): Timer3 runs in CTC mode with a period of one bit, and
 *     its compare ISR fires once per bit. The main loop turns each byte
 *     into a 16-bit frame word (start bit in bit 0, then data, parity and
 *     stop bits) and queues it. The ISR shifts the word out a bit at a
 *     time, programming OC3A to be set or cleared on the next match, so
 *     the line switches exactly on the bit grid. Its state lives in
 *     registers reserved for it (-ffixed-r2..r7): no pushes, 21 cycles per
 *     bit including interrupt entry, plus ~30 to fetch the next frame word.
 *     Bit times are whole timer ticks: at most 0.5 tick off per bit (0.08%
 *     at 115200).
 *
 * RX: input capture timestamps edges on ICP1. The capture ISR stores the
 *     16-bit timestamp and flips the edge polarity; its deadline is also
 *     the next edge. Because the polarity is flipped after every capture,
 *     captured edges always alternate falling/rising, so the level after
 *     each one follows from the current ICES1 setting and is not stored.
 *     The main loop turns the edge stream into bytes by sampling each bit
 *     in its middle.
 *
 * CTS flow control (P1_HW_FLOW) is checked by the compare ISR when it is
 * about to start a frame, so a frame in progress always completes.
 *
 * Both ISRs are hand written: they are what limits the usable baud rate.
 * At 16 MHz and 115200 baud a bit is 139 cycles. Counting from the
 * interrupt request (4 cycle response + jmp), the capture ISR flips the
 * polarity after 22 cycles and takes 53 in total (55 on overrun); the
 * compare ISR has OCR1A and COM1A updated after 50 cycles and takes 68 for
 * a data edge, 71 for a start edge, 66 at the end of a burst and 75 when
 * CTS holds a frame. The USART ISRs re-enable interrupts after 20 cycles
 * (see hwuart.c), so the worst case from a compare match to OCR1A being
 * updated is 20 + 55 + 50 = 125 cycles.
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

#if SU_TX_MODE != 0 && SU_TX_MODE != 3
#error SU_TX_MODE must be 0 or 3
#endif
#if SU_TX_MODE != 0 && !defined(SU_REGS_RESERVED)
#error SU_TX_MODE 3 needs r2-r7 reserved for the ISRs (-ffixed-r2 ... -ffixed-r7, see the Makefile)
#endif

#define RX_PORT    PORTD
#define RX_PIN     PIND
#define RX_BIT     PD4
#if SU_TX_MODE == 0
#define TX_PORT    PORTB
#define TX_DDR     DDRB
#define TX_BIT     PB5
#else
#define TX_PORT    PORTC
#define TX_DDR     DDRC
#define TX_BIT     PC6
#endif

/* TCCR1A values: OC1A set (line high) / cleared (line low) on match; the
 * codec's queue entries use the same codes */
#define COM_SET    SU_CTL_HIGH
#define COM_CLEAR  SU_CTL_LOW
#define CTL_START  SU_CTL_START
#if COM_SET != (_BV(COM1A1) | _BV(COM1A0)) || COM_CLEAR != _BV(COM1A1) || (CTL_START & (COM_SET | COM_CLEAR))
#error queue control codes do not match TCCR1A
#endif

/* debugging hook for the simulator: marks decoder runs */
#ifndef SU_TRACE_DECODE
#define SU_TRACE_DECODE()
#endif

/* how far ahead of "now" a transmission restarting from idle is scheduled */
#define TX_LEAD    32

/* GPIOR0 bits: transmitter state (SU_TX_MODE 0: neither RUN nor DRAIN =
 * idle, compare interrupt off) */
#define TXS_RUN    0          /* OCR1A holds the next queued edge */
#define TXS_DRAIN  1          /* OCR1A holds the end of the last frame's stop bits */
#define TXS_CTSARM 2          /* FLOW_AUTO: CTS is being obeyed */
#define TXS_IDLE   1          /* SU_TX_MODE 3: the ISR found nothing to start
                                 (ring empty or CTS) since the last start bit */

/* Edge ring (RX) and edge queue (TX) are 256-byte aligned rings of 2 and
 * 4 byte entries indexed by a byte offset, so the ISRs index with a single
 * register and wrap for free. The ISR-owned index of each lives in a GPIOR. */
#define E_HEAD     GPIOR2     /* written by the capture ISR */
#define Q_TAIL     GPIOR1     /* written by the compare ISR */

static struct
{
	uint8_t e[256];           /* {ICR1L, ICR1H} */
	uint8_t q[256];           /* mode 0: {delta lo, delta hi, TCCR1A | CTL_START, unused}
	                             mode 3: frame words, little endian */
} bufs __attribute__((aligned(256), section(".noinit")));

volatile uint8_t  su_e_tail;        /* edge ring tail, main loop */
volatile uint8_t  su_e_overrun;     /* set by the capture ISR when the ring is full */

/* ---- main loop state ---- */

static ring_t           tx_ring, rx_ring;
static volatile uint8_t errors;
static su_cfg_t         cfg;
static su_dec_t         dec;

#if SU_TX_MODE == 0
volatile uint8_t  su_q_head;        /* edge queue head, main loop */
volatile uint16_t su_tx_tail_ticks; /* stop bit time left after the last queued frame's last edge */
static su_enc_t   enc;

#define Q_ENTRIES_FREE() ((uint8_t)(255 - (uint8_t)(su_q_head - Q_TAIL)) / 4)
#else
/* Registers reserved for the TX ISR (the Makefile builds everything with
 * -ffixed for them):
 *   r3:r2  frame word being sent, next bit in bit 0
 *   r4     bits of it left to program
 *   r5     TCCR3A value: COM_CLEAR, with bit 6 set from the frame word
 *   r6     SREG, saved (also by the capture ISR)
 *   r7     the capture ISR's data register
 * The frame word queue is bufs.q, 128 words; the ISR's tail is in GPIOR1. */
#define TXW_TAIL   GPIOR1

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
#endif

/* ------------------------------------------------------------------------ */

#if SU_TX_MODE == 0
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

#else

/* The same with the registers reserved for the ISRs (see the TX ISR
 * below): SREG goes to r6, which the TX ISR also uses as scratch (neither
 * ISR can interrupt the other), and r7 is the data register. 45 cycles in
 * total instead of 53. */
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
#endif

#if SU_TX_MODE == 0

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
#if P1_HW_FLOW
		/* at a frame boundary, only go on while CTS is asserted (low) */
		"ldd  r25, Z+2"             "\n\t"
		"sbrs r25, 0"               "\n\t"   /* CTL_START */
		"rjmp 5f"                   "\n\t"
#if P1_HW_FLOW == FLOW_AUTO
		"sbis %[state], %[ctsarm]"  "\n\t"   /* not armed: CTS doesn't count */
		"rjmp 5f"                   "\n\t"
#endif
		"sbic %[ctspin], %[ctsbit]" "\n\t"
		"rjmp 6f"                   "\n\t"
	"5:"
#endif
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

#if P1_HW_FLOW
		/* CTS deasserted at a frame boundary: leave the frame queued and
		 * time the end of the previous one's stop bits (its start edge's
		 * delta) like an empty queue */
	"6:"	"lds  r24, %[ocrl]"         "\n\t"
		"ld   r25, Z+"              "\n\t"
		"add  r24, r25"             "\n\t"
		"ld   r25, Z"               "\n\t"
		"rjmp 7f"                   "\n\t"
#endif
		/* RUN, queue empty: time the end of the stop bits */
	"2:"	"lds  r24, %[ocrl]"         "\n\t"
		"lds  r25, %[ttail]"        "\n\t"
		"add  r24, r25"             "\n\t"
		"lds  r25, %[ttail]+1"      "\n\t"
	"7:"	"lds  r31, %[ocrh]"         "\n\t"
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
#if P1_HW_FLOW
#if P1_HW_FLOW == FLOW_AUTO
		"sbis %[state], %[ctsarm]"  "\n\t"
		"rjmp 8f"                   "\n\t"
#endif
		"sbic %[ctspin], %[ctsbit]" "\n\t"
		"rjmp 4f"                   "\n\t"
	"8:"
#endif
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
		[ctsarm]  "I" (TXS_CTSARM),
		[commask] "M" (~CTL_START & 0xFF),
		[comset]  "M" (COM_SET),
		[comclr]  "M" (COM_CLEAR),
		[nocie]   "M" (~_BV(OCIE1A) & 0xFF),
		[lead]    "I" (TX_LEAD),
		[qbuf]    "i" (bufs.q),
		[ctspin]  "I" (_SFR_IO_ADDR(GPIO_PINREG(P1_CTS_PIN))),
		[ctsbit]  "I" (GPIO_BIT(P1_CTS_PIN)),
		[qhead]   "i" (&su_q_head),
		[ttail]   "i" (&su_tx_tail_ticks)
	);
}

#else /* SU_TX_MODE == 3 */

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
		"cbi  %[state], %[idle]"    "\n\t"
	"3:"	"pop  r31"                  "\n\t"
		"pop  r30"                  "\n\t"
		"rjmp 1b"                   "\n\t"

		/* nothing to start: send one more 1 bit and look again */
	"4:"	"sbi  %[state], %[idle]"    "\n\t"
		"ldi  r30, 1"               "\n\t"
		"mov  r2, r30"              "\n\t"
		"mov  r4, r30"              "\n\t"
		"rjmp 3b"                   "\n\t"
		::
		[tccr3a]   "n" (_SFR_MEM_ADDR(TCCR3A)),
		[tail]     "I" (_SFR_IO_ADDR(TXW_TAIL)),
		[head]     "i" (&su_txw_head),
		[fbits]    "i" (&su_tx_fbits),
		[state]    "I" (_SFR_IO_ADDR(GPIOR0)),
		[idle]     "I" (TXS_IDLE),
		[ctsarm]   "I" (TXS_CTSARM),
		[qbuf]     "i" (bufs.q),
		[ctspin]   "I" (_SFR_IO_ADDR(GPIO_PINREG(P1_CTS_PIN))),
		[ctsbit]   "I" (GPIO_BIT(P1_CTS_PIN))
	);
}

#endif /* SU_TX_MODE */

/* ------------------------------------------------------------------------ */

void su_rx_frame(uint8_t data, uint8_t flags)
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
		errors |= CDC_CONTROL_LINE_IN_OVERRUNERROR;
	}
}

/* Encode bytes from the tx ring into the edge queue while there is room
 * for a worst case frame, then make sure the transmitter is running.
 *
 * At most TX_ENCODE_BATCH frames are encoded per call: this keeps main loop
 * iterations short so the receive side (which, unlike TX, cannot be held
 * off) is serviced often. When the CPU runs short at high rates, TX then
 * slows down by leaving gaps between frames instead of RX losing data.
 * With USB in the main loop an iteration can take longer than two frames at
 * 115200, so a batch of 2 left the line idle between batches even with
 * nothing else to do (91% of line rate on hardware; 100% with 4). */
#ifndef TX_ENCODE_BATCH
#define TX_ENCODE_BATCH 4
#endif

/* Receive has priority: while received edges are piling up (the decoder is
 * falling behind), don't queue more TX frames - fewer frames on the wire
 * also means fewer compare interrupts. */
#ifndef RX_BACKLOG_MAX
#if SU_TX_MODE == 3
#define RX_BACKLOG_MAX  20      /* edges; RX loses data in the simulator from ~36 */
#else
#define RX_BACKLOG_MAX  12      /* edges */
#endif
#endif

#if PROFILER
/* why encoding stopped, per call (see profiler.h) */
volatile uint16_t su_tx_stats[SU_TX_STATS];
#define TX_STAT(i) do { if (su_tx_stats[i] != 0xFFFF) su_tx_stats[i]++; } while (0)
#else
#define TX_STAT(i) do { } while (0)
#endif

#if SU_TX_MODE == 0

static void softuart_tx_start(void)
{
	/* checked once per call: a batch adds at most a few frames' worth of
	 * edges to the backlog, far from filling the edge ring */
	uint8_t n = ((uint8_t)(E_HEAD - su_e_tail) <= 2 * RX_BACKLOG_MAX) ? TX_ENCODE_BATCH : 0;

	TX_STAT(0);
	if (!n && !ring_empty(&tx_ring))
	  TX_STAT(1);

	for (uint8_t i = 0; i < n; i++)
	{
		if (ring_empty(&tx_ring))            { TX_STAT(3); break; }
		if (Q_ENTRIES_FREE() < SU_MAX_EDGES) { TX_STAT(4); break; }
		if (PROFILER && i == n - 1 && ring_count(&tx_ring) > 1)
		  TX_STAT(2);

		uint8_t h = su_encode(&cfg, &enc, ring_get(&tx_ring), bufs.q, su_q_head);

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
		if (!(GPIOR0 & (_BV(TXS_RUN) | _BV(TXS_DRAIN))) && t != su_q_head && !CTS_BLOCKED())
		{
			/* the first queued edge is a start bit; begin right away */
			TX_STAT(5);
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
	if (GPIOR0 & (_BV(TXS_RUN) | _BV(TXS_DRAIN)))
	  return false;
	return (Q_TAIL == su_q_head && ring_empty(&tx_ring)) || CTS_BLOCKED();
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

#else /* SU_TX_MODE == 3 */

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
	 * edges to the backlog, far from filling the edge ring */
	uint8_t n = ((uint8_t)(E_HEAD - su_e_tail) <= 2 * RX_BACKLOG_MAX) ? TX_ENCODE_BATCH : 0;

	TX_STAT(0);
	if (!n && !ring_empty(&tx_ring))
	  TX_STAT(1);

	for (uint8_t i = 0; i < n; i++)
	{
		uint8_t h = su_txw_head;
		if (ring_empty(&tx_ring))            { TX_STAT(3); break; }
		if ((uint8_t)(h + 2) == TXW_TAIL)    { TX_STAT(4); break; }
		if (PROFILER && i == n - 1 && ring_count(&tx_ring) > 1)
		  TX_STAT(2);
		uint16_t w = tx_word(ring_get(&tx_ring));
		bufs.q[h]     = w;
		bufs.q[h + 1] = w >> 8;
		asm volatile("" ::: "memory");      /* the word before the head */
		su_txw_head   = h + 2;
	}

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		if (TIMSK3 & _BV(OCIE3A))
		{
			/* Stop once the ISR has found nothing to start (queue empty or
			 * CTS holding it off): the last stop bit is programmed, and
			 * the ISR state is "one idle bit, then fetch", so a restart
			 * adds at least a bit of idle line and never cuts the stop
			 * bit short. */
			if ((GPIOR0 & _BV(TXS_IDLE)) && (TXW_TAIL == su_txw_head || CTS_BLOCKED()))
			  TIMSK3 &= ~_BV(OCIE3A);
		}
		else if (TXW_TAIL != su_txw_head && !CTS_BLOCKED())
		{
			TX_STAT(5);
			GPIOR0 &= ~_BV(TXS_IDLE);
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

#endif /* SU_TX_MODE */

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

#if SU_TX_MODE == 3
	/* Timer3: one bit per period, the smallest prescaler that fits */
	uint8_t cs3 = 0;
	while (cs3 < 4 && F_CPU / prescalers[cs3] / baud > 65535)
	  cs3++;
	uint16_t period = (F_CPU / prescalers[cs3] + baud / 2) / baud;
	/* 1.5 stop bits are sent as 2 */
	uint8_t fbits = 1 + cfg.data_bits + (cfg.parity != SU_PARITY_NONE) + (le->CharFormat ? 2 : 1);
#endif

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		uint8_t line = (RX_PIN >> RX_BIT) & 1;

#if SU_TX_MODE == 0
		TIMSK1 &= ~_BV(OCIE1A);
		GPIOR0 &= ~(_BV(TXS_RUN) | _BV(TXS_DRAIN));   /* keeps TXS_CTSARM */
		Q_TAIL  = su_q_head;
		su_enc_reset(&enc);
#else
		TIMSK3 &= ~_BV(OCIE3A);
		GPIOR0 &= ~_BV(TXS_IDLE);                     /* keeps TXS_CTSARM */
		TXW_TAIL    = su_txw_head;
		su_tx_fbits = fbits;
		tx_isr_idle();
		TCCR3B = 0;
		TCNT3  = 0;
		OCR3A  = period - 1;
		TCCR3A = COM_SET;
		TCCR3C = _BV(FOC3A);
		TCCR3B = _BV(WGM32) | (cs3 + 1);
#endif
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
#if SU_TX_MODE == 0
	TCCR1A = COM_SET;
	TCCR1C = _BV(FOC1A);             /* OC1A high: line idle */
#else
	TCCR1A = 0;
	tx_isr_idle();
	TCCR3A = COM_SET;
	TCCR3C = _BV(FOC3A);             /* OC3A high: line idle */
#endif
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
