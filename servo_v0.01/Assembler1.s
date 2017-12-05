
/*
 * Assembler1.s
 *
 * Created: 5.12.2017 15:55:22
 *  Author: fkla
 */ 

#define __SFR_OFFSET 0

;#include <avr/io.h>

.global transfer
.func transfer

transfer:
	push r16
	push r17

	ldi r17, 0b101
	ldi r16, 0b11011100

	sub r24, r16
	sbc r25, r17

	pop r17
	pop r16

	ret
	.endfunc
	
