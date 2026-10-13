/*
 * Soft UART decoder inner loop for the AVR: su_dec_run() from
 * softuart_codec.c (which has the reference C version and the description)
 * with the whole decoder state held in registers.
 *
 * void su_dec_run(su_dec_t* d, const su_cfg_t* c, const uint8_t* ring,
 *                 uint8_t tail, uint8_t head, uint8_t lvl_next, uint16_t now)
 *   d: r25:r24  c: r23:r22  ring: r21:r20  tail: r18  head: r16
 *   lvl_next: r14  now: r13:r12
 *
 * 'ring' must be 256-byte aligned (only its high byte is used).
 *
 * Registers (r2-r7 are left alone: the per-bit transmitter's ISR keeps its
 * state there, see softuart.c):
 *   r9:r8   bits                         r15  lvl (line level)
 *   r11:r10 mask                         r17  in_frame
 *   r14     lvl_next                     r18  tail
 *   r19     ring (high byte)             r21:r20  start
 *   r23:r22 &c->sample[0]                r25:r24  t (edge time / now)
 *   Y       next (pointer into sample[]) r27:r26  rel = t - start
 *   r31:r30 *next                        d: on the stack
 * An edge is being processed when tail != head; when they are equal, the
 * pass is the final one for 'now'.
 */
#include "softuart_codec.h"

#define D_IN_FRAME 0
#define D_LEVEL    1
#define D_NEXT     2
#define D_MASK     4
#define D_BITS     6
#define D_START    8

	.section .text.su_dec_run,"ax",@progbits
	.global su_dec_run
	.type   su_dec_run, @function
su_dec_run:
	push r8
	push r9
	push r10
	push r11
	push r14
	push r15
	push r17
	push r28
	push r29
	push r24                    /* d, for the write back */
	push r25

	/* load the state */
	movw r30, r24
	ld   r17, Z
	ldd  r15, Z+D_LEVEL
	ldd  r28, Z+D_NEXT
	ldd  r29, Z+D_NEXT+1
	ldd  r10, Z+D_MASK
	ldd  r11, Z+D_MASK+1
	ldd  r8,  Z+D_BITS
	ldd  r9,  Z+D_BITS+1
	mov  r19, r21
	ldd  r20, Z+D_START
	ldd  r21, Z+D_START+1
	subi r22, lo8(-SU_CFG_SAMPLE_OFS)
	sbci r23, hi8(-SU_CFG_SAMPLE_OFS)

.Lloop:
	/* next edge time, or 'now' as a final pseudo-edge */
	cp   r18, r16
	breq .Lnow
	mov  r30, r18
	mov  r31, r19
	ld   r24, Z+
	ld   r25, Z
	rjmp .Lsample
.Lnow:
	movw r24, r12

.Lsample:
	/* take the samples of the frame in progress that lie before t */
	tst  r17
	breq .Ledge
	movw r26, r24
	sub  r26, r20
	sbc  r27, r21
	ld   r30, Y
	ldd  r31, Y+1
	cp   r30, r26
	cpc  r31, r27
	brsh .Ledge                 /* next sample point not reached yet */

	cp   r28, r22               /* sampling the start bit? */
	cpc  r29, r23
	brne .Lfill
	tst  r15
	breq .Lfill
	clr  r17                    /* start bit gone by mid-bit: a glitch */
	rjmp .Ledge

.Lfill:
	tst  r15
	breq .Lfill0
.Lfill1:                        /* line high: set the bits */
	or   r8, r10
	or   r9, r11
	lsl  r10
	rol  r11
	adiw r28, 2
	ld   r30, Y
	ldd  r31, Y+1
	cp   r30, r26
	cpc  r31, r27
	brlo .Lfill1
	rjmp .Lfilled
.Lfill0:                        /* line low: just move on */
	lsl  r10
	rol  r11
	adiw r28, 2
	ld   r30, Y
	ldd  r31, Y+1
	cp   r30, r26
	cpc  r31, r27
	brlo .Lfill0
.Lfilled:
	adiw r30, 1                 /* the 0xFFFF sentinel: frame complete */
	brne .Ledge

	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25
	movw r24, r22               /* su_dec_finish_frame(c, bits) */
	sbiw r24, SU_CFG_SAMPLE_OFS
	movw r22, r8
	call su_dec_finish_frame
	clr  r1                     /* (the C ABI keeps it 0 anyway) */
	pop  r25
	pop  r24
	pop  r23
	pop  r22
	pop  r21
	pop  r20
	pop  r19
	pop  r18
	clr  r17

.Ledge:
	cp   r18, r16
	breq .Ldone
	subi r18, -2
	mov  r15, r14               /* level after this edge */
	ldi  r26, 1
	eor  r14, r26
	tst  r17
	brne .Lagain
	tst  r15
	brne .Lagain
	/* falling edge on an idle line: a start bit */
	ldi  r17, 1
	movw r20, r24
	movw r28, r22
	ldi  r26, 1
	mov  r10, r26
	clr  r11
	clr  r8
	clr  r9
.Lagain:
	rjmp .Lloop

.Ldone:
	pop  r31
	pop  r30
	st   Z, r17
	std  Z+D_LEVEL, r15
	std  Z+D_NEXT, r28
	std  Z+D_NEXT+1, r29
	std  Z+D_MASK, r10
	std  Z+D_MASK+1, r11
	std  Z+D_BITS, r8
	std  Z+D_BITS+1, r9
	std  Z+D_START, r20
	std  Z+D_START+1, r21

	pop  r29
	pop  r28
	pop  r17
	pop  r15
	pop  r14
	pop  r11
	pop  r10
	pop  r9
	pop  r8
	ret
	.size su_dec_run, .-su_dec_run
