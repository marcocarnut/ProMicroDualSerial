/*
 * Soft UART framing, independent of the hardware (unit tested on the host).
 *
 * Time is measured in 16-bit timer ticks. A frame is described by tables of
 * tick offsets from the leading edge of its start bit, computed once per line
 * coding with 1/256 tick precision so that odd ratios (e.g. 16 MHz / 115200 =
 * 138.89 ticks per bit) don't accumulate error across the frame.
 *
 * Encoding turns a byte into the line transitions ("edges") it needs, each
 * as a delay from the previous one, written straight into the transmit ISR's
 * queue. Decoding works the other way round from timestamped edges: each bit
 * is sampled in its middle by looking at the line level in effect then.
 *
 * Both run once per byte / per edge on an 8-bit CPU at up to ~115200 baud
 * full duplex, so they are written for speed: 16-bit arithmetic only, and
 * the decoder takes a whole batch of edges per call.
 */
#ifndef SOFTUART_CODEC_H
#define SOFTUART_CODEC_H

#ifndef __ASSEMBLER__
#include <stdint.h>
#include <stdbool.h>
#endif

/* parity, same numbering as CDC */
#define SU_PARITY_NONE   0
#define SU_PARITY_ODD    1
#define SU_PARITY_EVEN   2
#define SU_PARITY_MARK   3
#define SU_PARITY_SPACE  4

/* stop bits, same numbering as CDC */
#define SU_STOP_1        0
#define SU_STOP_1_5      1
#define SU_STOP_2        2

/* receive status flags, same bit values as CDC SERIAL_STATE */
#define SU_RX_BREAK      (1 << 2)
#define SU_RX_FRAMING    (1 << 4)
#define SU_RX_PARITY     (1 << 5)

/* start + 8 data + parity + stop */
#define SU_MAX_BITS      11
/* worst case: every bit boundary is a transition */
#define SU_MAX_EDGES     SU_MAX_BITS

/* Largest frame (in bit times) the timer must be able to represent. */
#define SU_MAX_FRAME_BITS 12

/*
 * Transmit queue format: a 256-byte ring of 4-byte entries
 * { delta lo, delta hi, ctl, unused }, indexed by a byte offset. 'delta' is
 * the time since the previous edge, 'ctl' says what the line does then. The
 * ctl codes are the ATmega32U4's TCCR1A values for "set OC1A on match" and
 * "clear OC1A on match", plus a flag (masked off by the ISR) marking the
 * first edge of a frame, whose delta counts from the previous frame's last
 * edge so that it lands exactly at the end of that frame's stop bits.
 */
#define SU_CTL_HIGH      0xC0
#define SU_CTL_LOW       0x80
#define SU_CTL_START     0x01

/* su_cfg_t field offsets, for the asm encoder and decoder (AVR layout) */
#define SU_CFG_PARITY_OFS     1
#define SU_CFG_BIT_START_OFS  3
#define SU_CFG_SAMPLE_OFS     (SU_CFG_BIT_START_OFS + 2 * SU_MAX_BITS)
#define SU_CFG_FRAME_LEN_OFS  (SU_CFG_SAMPLE_OFS + 2 * (SU_MAX_BITS + 1))
#define SU_CFG_FRAME_FRAC_OFS (SU_CFG_FRAME_LEN_OFS + 2)
#define SU_CFG_DATA_MASK_OFS  (SU_CFG_FRAME_FRAC_OFS + 1)
#define SU_CFG_ENC_OR_OFS     (SU_CFG_DATA_MASK_OFS + 7)
#define SU_CFG_ENC_PAR_OFS    (SU_CFG_ENC_OR_OFS + 2)
#define SU_CFG_PAR_LUT_OFS    (SU_CFG_ENC_PAR_OFS + 2)    /* 63: the last one ldd reaches */

/* su_enc_t field offsets */
#define SU_ENC_TAIL_OFS       0
#define SU_ENC_FRAC_OFS       2

#ifndef __ASSEMBLER__

typedef struct
{
	uint8_t  data_bits;                  /* 5..8 */
	uint8_t  parity;                     /* SU_PARITY_* */
	uint8_t  nbits;                      /* sampled bits: start, data, parity, first stop */
	uint16_t bit_start[SU_MAX_BITS];     /* offset of the start of each bit */
	uint16_t sample[SU_MAX_BITS + 1];    /* offset of the middle of each bit, then 0xFFFF */
	uint16_t frame_len;                  /* offset of the end of the (last) stop bit */
	uint8_t  frame_frac;                 /* fractional part of the frame length, 1/256 ticks */
	uint8_t  data_mask;                  /* (1 << data_bits) - 1 */
	uint16_t parity_mask;                /* parity bit in the sampled bits (0: none) */
	uint16_t stop_mask;                  /* stop bit in the sampled bits */
	uint16_t last_sample;                /* sample[nbits - 1] */

	/* for the asm encoder, which builds the frame without variable shifts:
	 * frame bits after the start bit = (data & data_mask) | enc_or, plus
	 * enc_par when the parity bit is 1 */
	uint16_t enc_or;                     /* stop bit and up: all ones */
	uint16_t enc_par;                    /* the parity bit (0: none) */
	uint8_t  par_lut;                    /* parity bit for data with an even (bit 0)
	                                        or odd (bit 1) number of ones */
} su_cfg_t;

/* Encoder state carried from frame to frame. */
typedef struct
{
	uint16_t tail;       /* ticks from the last edge to the end of the stop bits */
	uint8_t  frac;       /* accumulated sub-tick part of the frame lengths */
} su_enc_t;

typedef struct
{
	uint8_t         in_frame;
	uint8_t         level;   /* current line level */
	const uint16_t* next;    /* offset of the next bit to sample, in su_cfg_t.sample */
	uint16_t        mask;    /* that bit's position in 'bits' */
	uint16_t        bits;    /* sampled bits, bit i = level of frame bit i */
	uint16_t        start;   /* time of the start bit's leading edge */
} su_dec_t;

/* Called by su_dec_run() for each received frame; provided by the user of
 * the decoder. flags: SU_RX_*. For a break, data is 0 and not real data. */
void su_rx_frame(uint8_t data, uint8_t flags);

/* Fills 'c' for the given tick rate and line coding. data_bits outside
 * 5..8 is treated as 8. The caller picks timer_hz so that a frame fits in
 * 16 bits: timer_hz / baud * SU_MAX_FRAME_BITS < 65536. */
void su_cfg_init(su_cfg_t* c, uint32_t timer_hz, uint32_t baud,
                 uint8_t data_bits, uint8_t parity, uint8_t stop);

/* Resets the encoder for a frame that starts from an idle line. */
void su_enc_reset(su_enc_t* e);

/* Encodes 'data' into the queue ring 'q' at byte offset 'head' (room for
 * SU_MAX_EDGES entries is the caller's business) and returns the new head.
 * On the AVR 'q' must be 256-byte aligned.
 * Frame lengths alternate between rounding down and up so that back to back
 * frames keep the exact average rate. */
uint8_t su_encode(const su_cfg_t* c, su_enc_t* e, uint8_t data, uint8_t* q, uint8_t head);

void su_dec_reset(su_dec_t* d, uint8_t level);

/*
 * Decodes a batch of line transitions. 'ring' is a 256-byte ring of
 * little-endian 16-bit timestamps indexed by byte offset; the edges from
 * offset 'tail' up to 'head' are processed in order. Transitions alternate,
 * and 'level' is the line level after the first one. 'now' is a time by
 * which every transition up to then is in the batch: samples before it are
 * taken too (this completes a frame whose trailing bits are all 1s).
 * Completed frames are reported through su_rx_frame().
 *
 * Time is 16-bit: this must be called often enough that no more than
 * ~65536 - frame_len ticks pass between calls while a frame is in progress.
 */
void su_dec_run(su_dec_t* d, const su_cfg_t* c, const uint8_t* ring, uint8_t tail,
                uint8_t head, uint8_t level, uint16_t now);

/* internal: classifies a completed frame and calls su_rx_frame() */
void su_dec_finish_frame(const su_cfg_t* c, uint16_t bits);

#endif /* __ASSEMBLER__ */

#endif
