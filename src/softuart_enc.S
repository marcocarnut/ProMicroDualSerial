/*
 * Soft UART encoder for the AVR: su_encode() from softuart_codec.c (which
 * has the reference C version) in asm, about twice as fast.
 *
 * uint8_t su_encode(const su_cfg_t* c, su_enc_t* e, uint8_t data,
 *                   uint8_t* q, uint8_t head)
 *   c: r25:r24  e: r23:r22  data: r20  q: r19:r18 (256-byte aligned)  head: r16
 *   returns the new head in r24
 *
 * The frame bits after the start bit are built without variable shifts
 * from masks precomputed by su_cfg_init():
 *
 *   D = (data & data_mask) | enc_or | (parity bit ? enc_par : 0)
 *
 * (bits above the stop bit are ones), and bit i of T = D ^ (D << 1) is set
 * when the line changes at the start of frame bit i + 1. The loop walks T
 * alongside the bit_start table until no transitions are left.
 *
 * Registers:
 *   Y        e                         r22:r23  offset of the last edge
 *   Z        c, then &bit_start[i]     r18:r19  scratch / edge delta
 *   X        write position in q       r20      ctl of the last edge
 *   r25:r24  T                         r21      0x40 (toggles ctl)
 */
#include "softuart_codec.h"

	.section .text.su_encode,"ax",@progbits
	.global su_encode
	.type   su_encode, @function
su_encode:
	push r28
	push r29
	movw r28, r22                   ; Y = e
	movw r30, r24                   ; Z = c
	movw r26, r18                   ; X = q + head (q is 256-byte aligned)
	add  r26, r16

	/* the start bit: its delta is the previous frame's tail */
	ldd  r18, Y + SU_ENC_TAIL_OFS
	ldd  r19, Y + SU_ENC_TAIL_OFS + 1
	st   X+, r18
	st   X+, r19
	ldi  r18, SU_CTL_LOW | SU_CTL_START
	st   X+, r18
	inc  r26                        ; skip the pad byte, wrapping in the page

	/* frame length with the fractional carry; the tail is that minus the
	 * offset of the last edge, subtracted at the end */
	ldd  r18, Y + SU_ENC_FRAC_OFS
	ldd  r19, Z + SU_CFG_FRAME_FRAC_OFS
	add  r18, r19
	std  Y + SU_ENC_FRAC_OFS, r18
	ldd  r18, Z + SU_CFG_FRAME_LEN_OFS
	ldd  r19, Z + SU_CFG_FRAME_LEN_OFS + 1
	adc  r18, r1
	adc  r19, r1
	std  Y + SU_ENC_TAIL_OFS, r18
	std  Y + SU_ENC_TAIL_OFS + 1, r19

	/* D in r19:r18 */
	ldd  r18, Z + SU_CFG_DATA_MASK_OFS
	and  r20, r18
	ldd  r18, Z + SU_CFG_ENC_OR_OFS
	ldd  r19, Z + SU_CFG_ENC_OR_OFS + 1
	or   r18, r20
	ldd  r0, Z + SU_CFG_PARITY_OFS
	tst  r0
	breq 2f
	mov  r0, r20                    ; parity of the data into bit 0 of r0
	swap r0
	eor  r0, r20
	mov  r21, r0
	lsr  r21
	lsr  r21
	eor  r0, r21
	mov  r21, r0
	lsr  r21
	eor  r0, r21
	ldd  r21, Z + SU_CFG_PAR_LUT_OFS
	sbrc r0, 0
	lsr  r21
	sbrs r21, 0
	rjmp 2f
	ldd  r21, Z + SU_CFG_ENC_PAR_OFS
	or   r18, r21
	ldd  r21, Z + SU_CFG_ENC_PAR_OFS + 1
	or   r19, r21
2:
	/* T = D ^ (D << 1) */
	movw r24, r18
	lsl  r24
	rol  r25
	eor  r24, r18
	eor  r25, r19

	adiw r30, SU_CFG_BIT_START_OFS  ; Z = &bit_start[0]
	clr  r22                        ; last edge: the start bit's, at 0
	clr  r23
	ldi  r20, SU_CTL_LOW
	ldi  r21, SU_CTL_HIGH ^ SU_CTL_LOW

.Lloop:
	adiw r30, 2                     ; bit_start[i + 1] for bit i of T
	lsr  r25
	ror  r24
	brcs .Ledge
	sbiw r24, 0
	brne .Lloop
	rjmp .Ldone

.Ledge:
	ld   r18, Z
	ldd  r19, Z + 1
	sub  r18, r22                   ; delta from the last edge
	sbc  r19, r23
	add  r22, r18                   ; last = this one
	adc  r23, r19
	eor  r20, r21                   ; the line flips
	st   X+, r18
	st   X+, r19
	st   X+, r20
	inc  r26
	sbiw r24, 0
	brne .Lloop

.Ldone:
	ldd  r18, Y + SU_ENC_TAIL_OFS
	ldd  r19, Y + SU_ENC_TAIL_OFS + 1
	sub  r18, r22
	sbc  r19, r23
	std  Y + SU_ENC_TAIL_OFS, r18
	std  Y + SU_ENC_TAIL_OFS + 1, r19
	mov  r24, r26
	pop  r29
	pop  r28
	ret
	.size su_encode, . - su_encode
