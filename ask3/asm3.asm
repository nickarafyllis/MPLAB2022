.include "m328PBdef.inc"    ;ATmega328P microcontroller definitions
    
.org 0x0
rjmp reset
    
reset: 
;Init Stack Pointer
ldi r24, low(RAMEND)
out SPL, r24
ldi r24, high(RAMEND)
out SPL, r24

ldi r24, (1<<WGM11)|(1<<COM1A1)
sts TCCR1A, r24
ldi r24, (1<<WGM12)|(1<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10); ;init prescaler to 8
sts TCCR1B, r24

;Init PORTB as output 
ldi r24, 0b00111111
out DDRB, r24
    
main:
    clr r24
    in r24,PIND	    ;read buttons
    lsr r24 
    brcc PD0_on 
    lsr r24 
    brcc PD1_on 
    lsr r24 
    brcc PD2_on 
    lsr r24 
    brcc PD3_on 
    ldi r24,0
    sts OCR1AL,r24        
    sts OCR1AH,r24
    sts ICR1L,r24        
    sts ICR1H,r24
    rjmp main	    ;else jump to main

    
;set pwm frequency by changing ICR1
;Fpwm=Fclk/[N*(1+ICR1)]

PD0_on:
    ldi r24,low(7999)
    sts OCR1AL,r24  ;duty cycle to 50%
    ldi r24,high(7999)
    sts OCR1AH,r24
    ldi r24,low(15999)
    sts ICR1L,r24        
    ldi r24,high(15999)
    sts ICR1H,r24
loop_0:
    in r24,PIND	    ;read buttons
    andi r24,1	    ;ensure PD0 was unpressed
    breq loop_0
    rjmp main	    ;return to main
    
PD1_on:
    ldi r24,low(3999)
    sts OCR1AL,r24  ;duty cycle to 50%
    ldi r24,high(3999)
    sts OCR1AH,r24
    ldi r24,low(7999)
    sts ICR1L,r24       
    ldi r24,high(7999)
    sts ICR1H,r24
loop_1:
    in r24,PIND	    ;read buttons
    andi r24,2	    ;ensure PD1 was unpressed
    breq loop_1
    rjmp main	    ;return to main
PD2_on:
    ldi r24,low(1999)
    sts OCR1AL,r24  ;duty cycle to 50%
    ldi r24,high(1999)
    sts OCR1AH,r24
    ldi r24,low(3999)
    sts ICR1L,r24     
    ldi r24,high(3999)
    sts ICR1H,r24
loop_2:
    in r24,PIND	    ;read buttons
    andi r24,4	    ;ensure PD2 was unpressed
    breq loop_2
    rjmp main	    ;return to main
    
PD3_on:
    ldi r24,low(999)
    sts OCR1AL,r24        ;duty cycle to 50%
    ldi r24,high(999)
    sts OCR1AH,r24
    ldi r24,low(1999)
    sts ICR1L,r24       
    ldi r24,high(1999)
    sts ICR1H,r24
loop_3:
    in r24,PIND	    ;read buttons
    andi r24,8	    ;ensure PD3 was unpressed
    breq loop_3
    rjmp main	    ;return to main
