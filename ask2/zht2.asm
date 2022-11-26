.include "m328PBdef.inc"    ;ATmega328P microcontroller definitions
    
.org 0x0
rjmp reset
.org 0x2
rjmp ISR0

.equ FOSC_MHZ=16        ; Microcontroller operating frequency in MHz

.equ DEL_mS= 600           ; Delay in mS (Valid number from 1 to 4095)

.equ DEL_NU=FOSC_MHZ*DEL_mS    ;delay_mS routine: (1000*DEL_NU+6) cycles

.equ DEL_INT = 1000*FOSC_MHZ
reset: 
;Init Stack Pointer
ldi r24, low(RAMEND)
out SPL, r24
ldi r24, high(RAMEND)
out SPL, r24

;Init PORTC as output
ser r26
out DDRC, r26
    
;Interrupt on rising edge of INT0 pin
ldi r24,(1 << ISC01) | (1 << ISC00)
sts EICRA, r24
    
;Enable the INT0 interrupt(PD2)
ldi r24, (1 << INT0)
out EIMSK, r24

sei                ;Sets the Global Interrupt Flag  

loop1:
    clr r26
loop2:
    out PORTC, r26

    ldi r24, low(DEL_NU)    ;
    ldi r25, high(DEL_NU)    ; Set delay (number of cycles)
    rcall delay_mS        ;
    
    inc r26

    cpi r26, 31           ;compare r26 with 16
    breq loop1
    rjmp loop2
;delay of 1000*F1+6 cycles (almost equal to 1000*F1 cycles)
Delay_mS:

;total delay of next 4 instruction group = 1+(249*4-1) = 996 cycles
    ldi r23, 249        ;(1 cycle)
loop_inn:
    dec r23             ;1 cycle
    nop			;1 cycle
    brne loop_inn       ;1 or 2 cycles

    sbiw r24, 1		;2 cycles
    brne delay_ms       ;1 or 2 cycles
    
    ret			;4 cycles
    
ISR0:
    push r24            ;
    push r25		;
    push r26		;
    in r24, SREG        ;Save r24, r25, r26, SREG
    push r24		;

    in r24, PINB	;get state of PORTB 
    ser r25		;negative logic for LEDs
    ldi r26, 7		;initialise counter

;check how many buttons are pressed 
check:			
    dec r26
    breq out_c		;if r26 = 0 we checked RB5-RB0, output to PORTC.
    lsr r24		;check if the button is pressed (negative logic)
    brcs check		;if c = 1 the button is not pressed, check next button 
    lsl r25		;else, adjust output. 
    rjmp check		;check next button.    
;output to PORTC
out_c:
    com r25		    ;complement output
    out PORTC, r25	    ;output to PORTC
    ldi r24, low(DEL_INT)    ;
    ldi r25, high(DEL_INT)   ; Set delay (number of cycles)
    rcall delay_mS      

    pop r24             ;
    out SREG, r24       ; Restore r24, r25, r26, SREG
    pop r26		;
    pop r25		;
    pop r24             ;
    reti


