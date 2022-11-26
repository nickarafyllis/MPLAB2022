.include "m328PBdef.inc"	;ATmega328P microcontroller definitions

.org 0x0
rjmp reset
.org 0x4
rjmp ISR1
    
.equ FOSC_MHZ=16		; Microcontroller operating frequency in MHz

.equ DEL_mS=500			; Delay in mS (Valid number from 1 to 4095)

.equ DEL_NU=FOSC_MHZ*DEL_mS	;delay_mS routine: (1000*DEL_NU+6) cycles

.equ DEL_INT = 5*16

reset: 
;Init Stack Pointer
ldi r24, low(RAMEND)
out SPL, r24
ldi r24, high(RAMEND)
out SPL, r24
    
;initialise int1 counter to zero
clr r27

;Init PORTB, PORTC as output
ser r26
out DDRB, r26
out DDRC, r26

;Interrupt on rising edge of INT1 pin
ldi r24,(1 << ISC11) | (1 << ISC10)
sts EICRA, r24
    
;Enable the INT1 interrupt(PD3)
ldi r24, (1 << INT1)
out EIMSK, r24

sei				;Sets the Global Interrupt Flag    

loop1:
	clr r26
loop2:
	out PORTB, r26

	ldi r24, low(DEL_NU)	;
	ldi r25, high(DEL_NU)	; Set delay (number of cycles)
	rcall delay_mS		;
	
	inc r26

	cpi r26, 16		;compare r26 with 16
	breq loop1
	rjmp loop2
;delay of 1000*F1+6 cycles (almost equal to 1000*F1 cycles)
delay_mS:

;total delay of next 4 instruction group = 1+(249*4-1) = 996 cycles
	ldi r23, 249		;(1 cycle)
loop_inn:
	dec r23			;1 cycle
	nop			;1 cycle
	brne loop_inn		;1 or 2 cycles

	sbiw r24, 1		;2 cycles
	brne delay_mS		;1 or 2 cycles
	
	ret			;4 cycles
	
ISR1:
    push r24			;
    in r24, SREG		; Save r24, SREG
    push r24			;
check_f1:
    ldi r24,(1 << INTF1)	; EIFR.1 <- 1
    out EIFR, r24
    ldi r24, low(DEL_INT)	;
    ldi r25, high(DEL_INT)	; Set delay (number of cycles)
    rcall delay_mS		; delay 5msec
    in r24, EIFR		; read EIFR.1
    andi r24, 2
    brne check_f1		; if EIFR.1 = 0 continue, else jump to check_f1    
    in r24,PIND			 
    andi r24, 128		; Check PD7
    breq tag_1			; if PD7 = 0, then dont increment counter
    cpi r27, 31			; if counter = 31, reset INT1 counter
    breq reset_counter
count:
    inc r27 
tag_1:
    out PORTC, r27		; Show counter value on leds PC4-PC0.				;
    pop r24			;
    out SREG, r24		; Restore r24, SREG
    pop r24			;
    reti
reset_counter:
    clr r27			;reset counter
    rjmp tag_1			;jump to tag_1


