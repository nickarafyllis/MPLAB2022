.include "m328PBdef.inc"
reset:
    ldi r24, low(RAMEND)
    out SPL, r24
    ldi r24, high(RAMEND)
    out SPH, r24
    
init:
    clr r16
    out DDRD, r16   ;Initialize PORTD as output
    ldi r16,1
    out PORTD, r16  ;Set Lsb = 1 (starting position)
    clt			;Set T flag to 0(T=0:left,T=1:right)

main:
    brtc left		;if T = 0, shift left, else shift right 
    rjmp right

right:
    ldi r24,low(500)	;set pair of registers r25:r24 to 500 for 500ms(0.5sec) delay
    ldi r25,high(500)
    rcall wait_x_msec	;0.5 sec delay
    lsr r16
    out PORTD, r16
    cpi r16,1
    breq changetoleft
    rjmp main
    
left:
    ldi r24,low(500)	;set pair of registers r25:r24 to 500 for 500ms(0.5sec) delay
    ldi r25,high(500)
    rcall wait_x_msec	;0.5 sec delay
    lsl r16
    out PORTD, r16
    cpi r16,128
    breq changetoright
    rjmp main

changetoright:
    set			;set T flag
    ldi r24 , low(1000) ;load r25:r24 with 1000
    ldi r25 , high(1000)
    rcall wait_x_msec   ;1 sec delay
    rjmp main    
   
changetoleft:
    clt		        ;clear T flag
    ldi r24 , low(1000) ;load r25:r24 with 1000
    ldi r25 , high(1000)
    rcall wait_x_msec   ;1 sec delay
    rjmp main
    
wait_x_msec:
loop_1:
    ldi r26, 99	;1
loop_2:		    ;10*99=990 cycles
    rcall wait4	    ;3+4 = 7
    dec r26	    ;1
    brne loop_2	    ;2 
    sbiw r25:r24, 1 ;2 cycles
    brne stall	    ;if value > 0 then 2 cycles, if value = 0 then 1 cycle.
    ret		    ;4+3
stall:		    ;we need 6 more cycles to reach 1000 if the branch is taken 
    nop		    ;1
    nop		    ;1
    nop		    ;1
    nop		    ;1
    rjmp loop_1	    ;2 ,total 6 cycles
    
wait4:
    ret		    ;4 cycles


