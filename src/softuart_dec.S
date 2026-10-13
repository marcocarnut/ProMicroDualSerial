/*
 * Soft UART decoder for the AVR: su_dec_run() from softuart_codec.c (which
 * has the reference C version and the description) in asm.
 *
 * void su_dec_run(su_dec_t* d, const su_cfg_t* c, const uint8_t* ring,
 *                 uint8_t tail, uint8_t head, uint8_t lvl_next, uint16_t now)
 *   d: r25:r24  c: r23:r22  ring: r21:r20  tail: r18  head: r16
 *   lvl_next: r14  now: r13:r12
 *
 * 'ring' must be 256-byte aligned (only its high byte is used).
 *
 * The decoder is a state machine whose state is the code position: idle or
 * in a frame, line high or low. Received edges alternate, so each edge
 * moves to the state of the other level and no level is kept in a register
 * or tested. Per edge that doesn't complete a sample this is ~18 cycles.
 *
 * Registers (r2-r7 are left alone: the per-bit transmitter's ISR keeps its
 * state there, see softuart.c). Everything that lives across the
 * su_dec_finish_frame() call is in call-saved registers:
 *   X        ring + tail                 r9       ring (high byte)
 *   r11:r10  start                       r8       tail, across the call
 *   r13:r12  now                         r15:r14  &c->sample[0]
 *   r16      head                        Y        next (pointer into sample[])
 *   r21:r20  bits                        r23:r22  mask
 *   r25:r24  t (edge time)               r19:r18  rel = t - start
 *   r31:r30  *next                       d: on the stack
 */
#include "softuart_codec.h"

#define D_IN_FRAME 0
#define D_LEVEL    1
#define D_NEXT     2
#define D_MASK     4
#define D_BITS     6
#define D_START    8

/* report the frame in r21:r20 and set up for the next one */
.macro FINISH
	mov  r8, r26
	movw r24, r14
	sbiw r24, SU_CFG_SAMPLE_OFS
	movw r22, r20
	call su_dec_finish_frame
	mov  r26, r8
	mov  r27, r9
	movw r28, r14
	ldi  r22, 1
	ldi  r23, 0
	ldi  r20, 0
	ldi  r21, 0
.endm

/* rel = \t - start; branch to \nosample when the next sample point isn't
 * before it */
.macro REL_CMP t, nosample
	movw r18, \t
	sub  r18, r10
	sbc  r19, r11
	ld   r30, Y
	ldd  r31, Y+1
	cp   r30, r18
	cpc  r31, r19
	brsh \nosample
.endm

/* take the samples before rel at a high (\high = 1) or low line, ending
 * with Z = 0 if the frame is complete */
.macro FILL high
1:
.if \high
	or   r20, r22
	or   r21, r23
.endif
	lsl  r22
	rol  r23
	adiw r28, 2
	ld   r30, Y
	ldd  r31, Y+1
	cp   r30, r18
	cpc  r31, r19
	brlo 1b
	adiw r30, 1                 /* the 0xFFFF sentinel: frame complete */
.endm

/* the next edge into r25:r24, or branch to \none when there is none */
.macro NEXT_EDGE none
	cp   r26, r16
	breq \none
	ld   r24, X+
	ld   r25, X+
	mov  r27, r9                /* undo the carry of X+ at the end of the page */
.endm

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
	push r28
	push r29
	push r24                    /* d, for the write back */
	push r25

	mov  r9, r21
	mov  r26, r18
	mov  r27, r21
	mov  r19, r14
	subi r22, lo8(-SU_CFG_SAMPLE_OFS)
	sbci r23, hi8(-SU_CFG_SAMPLE_OFS)
	movw r14, r22

	movw r30, r24
	ld   r18, Z
	ldd  r0,  Z+D_LEVEL
	ldd  r28, Z+D_NEXT
	ldd  r29, Z+D_NEXT+1
	ldd  r22, Z+D_MASK
	ldd  r23, Z+D_MASK+1
	ldd  r20, Z+D_BITS
	ldd  r21, Z+D_BITS+1
	ldd  r10, Z+D_START
	ldd  r11, Z+D_START+1

	/* with edges to process, the line level before the first one is the
	 * opposite of lvl_next */
	cp   r26, r16
	breq 1f
	ldi  r24, 1
	eor  r19, r24
	mov  r0, r19
1:
	tst  r18
	breq 2f
	sbrc r0, 0
	rjmp .LFH
	rjmp .LFL
2:	sbrc r0, 0
	rjmp .LIH
	rjmp .LIL

	/* (in reach of the hot loops' branches) */
.LFL_now_t:
	rjmp .LFL_now
.LFH_now_t:
	rjmp .LFH_now
.Lexit_IH_t:
	rjmp .Lexit_IH
.Lexit_IL_t:
	rjmp .Lexit_IL

	/* idle, line low (a break, or after a glitch): wait for it to rise */
.LIL:
	cp   r26, r16
	breq .Lexit_IL_t
	subi r26, -2
	/* fall through */

	/* idle, line high: the next (falling) edge starts a frame */
.LIH:
	cp   r26, r16
	breq .Lexit_IH_t
	ld   r10, X+
	ld   r11, X+
	mov  r27, r9
	movw r28, r14
	ldi  r22, 1
	ldi  r23, 0
	ldi  r20, 0
	ldi  r21, 0
	/* fall through */

	/* in a frame, line low: samples before the next (rising) edge are 0s */
.LFL:
	NEXT_EDGE .LFL_now_t
	REL_CMP r24, .LFH
	FILL 0
	brne .LFH
	rjmp .LFL_finish

	/* in a frame, line high: samples before the next (falling) edge are 1s */
.LFH:
	NEXT_EDGE .LFH_now_t
	REL_CMP r24, .LFL
	cp   r28, r14               /* the start bit's sample: */
	cpc  r29, r15
	breq .LFH_glitch
	FILL 1
	brne .LFL
	movw r10, r24               /* the falling edge starts the next frame */
	FINISH
	rjmp .LFL
.LFH_glitch:                    /* start bit gone by mid-bit: a glitch, and */
	movw r10, r24               /* the falling edge starts a new frame */
	rjmp .LFL

.LFL_finish:
	FINISH                      /* the rising edge leaves the line idle */
	rjmp .LIH

	/* no more edges: take the samples before 'now' */
.LFL_now:
	REL_CMP r12, .Lexit_FL
	FILL 0
	brne .Lexit_FL
	FINISH
	rjmp .Lexit_IL
.Lexit_FL:
	ldi  r18, 1
	ldi  r19, 0
	rjmp .Lstore

.LFH_now:
	REL_CMP r12, .Lexit_FH
	cp   r28, r14
	cpc  r29, r15
	breq .Lexit_IH              /* glitch */
	FILL 1
	brne .Lexit_FH
	FINISH
	/* fall through */
.Lexit_IH:
	ldi  r18, 0
	ldi  r19, 1
	rjmp .Lstore
.Lexit_IL:
	ldi  r18, 0
	ldi  r19, 0
	rjmp .Lstore
.Lexit_FH:
	ldi  r18, 1
	ldi  r19, 1
.Lstore:
	pop  r31
	pop  r30
	st   Z, r18
	std  Z+D_LEVEL, r19
	std  Z+D_NEXT, r28
	std  Z+D_NEXT+1, r29
	std  Z+D_MASK, r22
	std  Z+D_MASK+1, r23
	std  Z+D_BITS, r20
	std  Z+D_BITS+1, r21
	std  Z+D_START, r10
	std  Z+D_START+1, r11

	pop  r29
	pop  r28
	pop  r15
	pop  r14
	pop  r11
	pop  r10
	pop  r9
	pop  r8
	ret
	.size su_dec_run, .-su_dec_run
