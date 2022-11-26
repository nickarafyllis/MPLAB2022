.include "m328PBdef.inc"    ;ATmega328P microcontroller definitions
    
.org 0x0
rjmp reset
;.def TOP =256 or TOP = 2^16
    
reset: 
;Init Stack Pointer
ldi r24, low(RAMEND)
out SPL, r24
ldi r24, high(RAMEND)
out SPL, r24

ldi r24, (1<<WGM10)|(1<<COM1A1)
sts TCCR1A, r24
ldi r24, (1<<WGM12)|(1<<CS11)
sts TCCR1B, r24

;Init PORTB as output 
ldi r24, 0b00111111
out DDRB, r24
;Assuming TOP is known
;Init duty cycle to 50%
ldi ZH,high(Table*2)	;save address of table to Z register
ldi ZL,low(Table*2)	;
adiw ZL,12		;point to byte of 50% duty cycle value    
lpm			;load low byte value
mov r24, r0    
sts OCR1AL,r24		;save low byte value

;init r25(duty cycle % counter) to 10
ldi r25,10	
    
main:
    in r24,PIND		;read buttons
    andi r24,6		;get values of PD1,PD2
    cpi r24,4		;if PD1 is pressed (00000100) jump to increase
    breq increase
    cpi r24,2		;else if PD2 is pressed(00000010) jump to decrease
    breq decrease
    rjmp main		;else jump to main
increase:
    in r24,PIND		;
    andi r24,2		;if PD1 still pressed wait for it to be unpressed
    breq increase	;
    cpi r25,16		;if r25=16(98% duty cycle) dont increase further
    breq main
    adiw ZL,2		;point to byte of x+8% duty cycle value 
    lpm			;load byte value
    mov r24, r0 
    sts OCR1AL,r24	;save byte value
    inc r25		;increment duty cycle % counter
    rjmp main		;return to main
decrease:
    in r24,PIND		;
    andi r24,4		;if PD2 still pressed wait for it to be unpressed
    breq decrease	;
    cpi r25,4		;if r25=4(2% duty cycle) dont decrease further
    breq main
    sbiw ZL,2		;point to byte of x-8% duty cycle value   
    lpm			;load low byte value
    mov r24, r0 
    sts OCR1AL,r24	;save low byte value
    dec r25		;decrement duty cycle % counter
    rjmp main		;return to main

Table:		;organized by word, enter values here
.dw 0x0005,0x001A,0x002E,0x0042,0x0057	    ;2,10,18,26,34,
.dw 0x006B,0x0080,0x0094,0x00A8,0x00BD	    ;42,50,58,66,74,
.dw 0x00D1,0x00E5,0x00FA		    ;82,90,98


