.include "m328PBdef.inc"
reset:
    ldi r24, low(RAMEND)
    out SPL, r24
    ldi r24, high(RAMEND)
    out SPH, r24

start:
    ldi r24,low(17)
    ldi r25,high(17)
    rcall wait_x_msec
    rjmp start
    
wait_x_msec:
    push r24
    push r25
    push r26
    push r27
    ldi r27,17
loop_1:
    ldi r26, 99	;1
loop_2:		    ;10*99=990 cycles
    rcall wait4	    ;3+4 = 7
    dec r26	    ;1
    brne loop_2	    ;2 
    sbiw r25:r24, 1 ;2 cycles
    brne stall	    ;if value > 0 then 2 cycles, if value = 0 then 1 cycle.
    pop r27
    pop r26
    pop r25
    pop r24
    ret		    ;4+3
stall:		    ;we need 6 more cycles to reach 1000 if the branch is taken 
    cpi r27,0	    ;1
    breq stall_2    ;2/1
    subi r27,1	    ;1
    rjmp loop_1	    ;2 ,total 5 cycles(while r27 =/= 0)
stall_2:
    nop		    ;1
    rjmp loop_1	    ;2
wait4:
    ret		    ;4 cycles


