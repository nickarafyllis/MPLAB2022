.include "m328PBdef.inc"    ;ATmega328P microcontroller definitions
    
.org 0x0
rjmp reset
.org 0x4
rjmp ISR1

.equ FOSC_MHZ=16        ; Microcontroller operating frequency in MHz

.equ DEL_mS=500          ; Delay in mS (Valid number from 1 to 4095)

.equ DEL_NU=FOSC_MHZ*DEL_mS    ;delay_mS routine: (1000*DEL_NU+6) cycles
    
.equ DEL_INT=500*FOSC_MHZ
.equ DEL_INT2=4000*FOSC_MHZ

reset: 
;Init Stack Pointer
ldi r24, low(RAMEND)
out SPL, r24
ldi r24, high(RAMEND)
out SPL, r24

;Init PORTC as output
ser r26
out DDRB, r26
    
;Interrupt on rising edge of INT1 pin
ldi r24,(1 << ISC11) | (1 << ISC10)
sts EICRA, r24
    
;Enable the INT1 interrupt(PD3)
ldi r24, (1 << INT1)
out EIMSK, r24

sei		     ;Sets the Global Interrupt Flag  
;init
clr r22           ;Initialize helper variable
    
main:		     ;infinite loop
    rjmp main

;delay of 1000*F1+6 cycles (almost equal to 1000*F1 cycles)
delay_mS:

;total delay of next 4 instruction group = 1+(249*4-1) = 996 cycles
    ldi r23, 249	 ;(1 cycle)
loop_inn:
    dec r23              ;1 cycle
    nop			 ;1 cycle
    brne loop_inn        ;1 or 2 cycles

    sbiw r24, 1		 ;2 cycles
    brne delay_ms        ;1 or 2 cycles
    
    ret
    
ISR1:
check_f1:
    ldi r24,(1 << INTF1)	; EIFR.1 <- 1
    out EIFR, r24
    ldi r24, low(5*16)	;
    ldi r25, high(5*16)	; Set delay (number of cycles)
    rcall delay_mS		; delay 5msec
    in r24, EIFR		; read EIFR.1
    andi r24, 2
    brne check_f1		; if EIFR.1 = 0 continue, else jump to check_f1    
    sei			;re enable interrupts
    cpi r22, 0		;if r22 == 0(1st interrupt occured)
    breq lsb_on	;jump to lsb_on 
    ser r24			;else (an interrupt occured while a previous one hadnt finished)
    out PORTB, r24		;Turn all LEDs on
    ldi r24, low(DEL_INT)       ;
    ldi r25, high(DEL_INT)      ; Set delay (number of cycles)
    rcall delay_mS		; delay 0.5sec
lsb_on:   
    inc r22			;increase additional interrupt counter 
    ldi r24, 1            
    out PORTB, r24		;LSB LED on
    ldi r24, low(DEL_INT2)	;
    ldi r25, high(DEL_INT2)	; Set delay (number of cycles)
    rcall delay_mS      ;delay for 4 seconds
    clr    r24          ;        
    out PORTB, r24      ;turn off the LED
    clr r22		;reset additional interrupt counter
    reti		;done, return to main