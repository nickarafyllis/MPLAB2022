.include "m328PBdef.inc"    ;ATmega328P microcontroller definitions
.org 0x0
rjmp reset
.org 0x4
rjmp ISR1
.org 0x1A
rjmp ISR_TIMER1_OVF

    
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

;init timer
    ldi r24,(1 << TOIE1)    ;enable interrupt of TCNT1
    sts TIMSK1, r24	    ;for timer1
    ;16Mhz/1024=15625Hz
    ldi r24,(1 << CS12)|(0 << CS11)|(1 << CS10)	;CK/1024
    sts TCCR1B, r24

    
sei		     ;Sets the Global Interrupt Flag  

;Init PORTB as output
ser r26
out DDRB, r26

main: 
    in r24,PINC
    andi r24,0x20   
    brne main	    ;Check PC5,if its not pressed branch to main
unpressed:
    in r24,PINC	    ;
    andi r24,0x20   ;ensure that PC5 is unpressed
    breq unpressed  ;
    in r24,PORTB    ;if PC5 is pressed check PORTB 
    andi r24,1	    ;check if PB0 of PORTB is on
    breq lsb_on	    ;if not, turn PB0 on for 4 seconds
    rcall _refresh   ;else refresh(0.5 seconds PB0-5 on, 3.5 seconds PB0 on)
    rjmp main
lsb_on:
    rcall lsb_on_4    
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
    push r24			;
    in r24, SREG		; Save r24, SREG
    push r24			;
check_f1:
    ldi r24,(1 << INTF1)    ; EIFR.1 <- 1
    out EIFR, r24
    ldi r24, low(5)   ;
    ldi r25, high(5)  ; Set delay (number of cycles)
    rcall delay_mS      ; delay 5msec
    in r24, EIFR        ; read EIFR.1
    andi r24, 2
    brne check_f1       ; if EIFR.1 = 0 continue, else jump to check_f1    
    in r24,PORTB	;read output
    andi r24,1		;check PB0
    brne refresh	;if PB0 is on, jump to refresh 
    rcall lsb_on_4	;else turn PB0 on for 4 seconds
    rjmp reset1
refresh:
    rcall _refresh	;start refresh
reset1:    
    pop r24			;
    out SREG, r24		; Restore r24, SREG
    pop r24			;
    reti
    
ISR_TIMER1_OVF:
    push r24			;
    in r24, SREG		; Save r24, SREG
    push r24			;
    in r24,PORTB    ;check PORTB
    andi r24,4
    breq leds_off   ;if only PB0 is on turn it off
    ldi r24,1	    ;else if PB0-5 are on (refresh occurred)
    out PORTB,r24   ;Turn PB1-5 off, leave PB0 on and delay for 3.5 more seconds
    ;we need 3.5 second extra delay, set timer1 again 
    ;3.5*15625=54687
    ;overflow happens in 65536
    ;so we set TCNT1 to 65536-54687=10849
    ldi r24,high(10849)
    sts TCNT1H,r24
    ldi r24,low(10849)
    sts TCNT1L,r24
    rjmp reset2
leds_off:
    ldi r24,0
    out PORTB,r24
reset2:  
    pop r24			;
    out SREG, r24		; Restore r24, SREG
    pop r24			;
    reti

;turns PB0 on, sets timer1 for 4 second delay    
lsb_on_4:
    push r24			;
    in r24, SREG		; Save r24, SREG
    push r24			;
    ;4*15625=62500
    ;overflow happens in 65536
    ;so we set TCNT1 to 65536-62500=3036
    ldi r24,high(3036)
    sts TCNT1H,r24
    ldi r24,low(3036)
    sts TCNT1L,r24
    ldi r24,1		;turn PB0 on for 4 seconds.
    out PORTB,r24
    pop r24			;
    out SREG, r24		; Restore r24, SREG
    pop r24			;
    ret

;turns PB0-5 on, sets timer1 for 0.5 second delay
_refresh:
    push r24			;
    in r24, SREG		; Save r24, SREG
    push r24			;
    ;0.5*15625=7812
    ;overflow happens in 65536
    ;so we set TCNT1 to 65536-7812=57724
    ldi r24,high(57724)
    sts TCNT1H,r24
    ldi r24,low(57724)
    sts TCNT1L,r24
    ldi r24, 0x3F
    out PORTB, r24	;turn PB0-5 on for 4 seconds.
    pop r24			;
    out SREG, r24		; Restore r24, SREG
    pop r24			;
    ret


