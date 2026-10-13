/*
 * Soft UART framing, independent of the hardware (unit tested on the host).
 *
 * Time is measured in timer ticks. A frame is described by tables of tick
 * offsets from the leading edge of its start bit, computed once per line
 * coding with 1/256 tick precision so that odd ratios (e.g. 16 MHz / 115200 =
 * 138.89 ticks per bit) don't accumulate error across the frame.
 *
 * Encoding turns a byte into the list of line transitions ("edges") it
 * needs, each as a delay from the previous one. Decoding works the other
 * way round from timestamped edges: each bit is sampled in its middle by
 * looking at the line level that was in effect at that time.
 */
#ifndef SOFTUART_CODEC_H
#define SOFTUART_CODEC_H

#include <stdint.h>
#include <stdbool.h>

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

typedef struct
{
	uint8_t  data_bits;                  /* 5..8 */
	uint8_t  parity;                     /* SU_PARITY_* */
	uint8_t  nbits;                      /* sampled bits: start, data, parity, first stop */
	uint16_t bit_start[SU_MAX_BITS];     /* offset of the start of each bit */
	uint16_t sample[SU_MAX_BITS];        /* offset of the middle of each bit */
	uint16_t frame_len;                  /* offset of the end of the (last) stop bit */
	uint8_t  frame_frac;                 /* fractional part of the frame length, 1/256 ticks */
} su_cfg_t;

/* Edge produced by the encoder: 'delta' ticks after the previous edge the
 * line goes to 'level'. The first edge of each frame (the start bit) has
 * 'start' set and its delta counts from the previous frame's last edge, so
 * that it lands exactly at the end of that frame's stop bits. */
typedef struct
{
	uint16_t delta;
	uint8_t  level;
	uint8_t  start;
} su_edge_t;

/* Encoder state carried from frame to frame. */
typedef struct
{
	uint16_t tail;       /* ticks from the last edge to the end of the stop bits */
	uint8_t  frac;       /* accumulated sub-tick part of the frame lengths */
} su_enc_t;

typedef struct
{
	uint8_t  in_frame;
	uint8_t  level;      /* current line level */
	uint8_t  idx;        /* next bit to sample */
	uint16_t bits;       /* sampled bits, bit i = level of frame bit i */
	uint32_t start;      /* time of the start bit's leading edge */
} su_dec_t;

typedef void (*su_rx_cb_t)(uint8_t data, uint8_t flags);

/* Largest frame (in bit times) the timer must be able to represent. */
#define SU_MAX_FRAME_BITS 12

/* Fills 'c' for the given tick rate and line coding. data_bits outside
 * 5..8 is treated as 8. The caller picks timer_hz so that a frame fits in
 * 16 bits: timer_hz / baud * SU_MAX_FRAME_BITS < 65536. */
void su_cfg_init(su_cfg_t* c, uint32_t timer_hz, uint32_t baud,
                 uint8_t data_bits, uint8_t parity, uint8_t stop);

/* Resets the encoder for a frame that starts from an idle line. */
void su_enc_reset(su_enc_t* e);

/* Encodes 'data' into out[] (at most SU_MAX_EDGES entries) and returns the
 * edge count. The first edge's delta is the previous frame's tail; frame
 * lengths alternate between rounding down and up so that back to back
 * frames keep the exact average rate. */
uint8_t su_encode(const su_cfg_t* c, su_enc_t* e, uint8_t data, su_edge_t* out);

void su_dec_reset(su_dec_t* d, uint8_t level);

/* Feed one line transition at time t, to the given level. Edges must be fed
 * in order. Completed frames are reported through cb. */
void su_dec_edge(su_dec_t* d, const su_cfg_t* c, uint32_t t, uint8_t level, su_rx_cb_t cb);

/* Tell the decoder that no transition happened before 'now', so samples
 * before then can be taken (this is what completes a frame whose trailing
 * bits are all 1s). */
void su_dec_time(su_dec_t* d, const su_cfg_t* c, uint32_t now, su_rx_cb_t cb);

#endif
