.include "m328PBdef.inc"
reset:
    ldi r24, low(RAMEND)
    out SPL, r24
    ldi r24, high(RAMEND)
    out SPH, r24

start:
    ldi r24,low(1000)
    ldi r25,high(1000)
    rcall wait_x_msec
    rjmp start

wait_x_msec:
loop_1:
    ldi r26, 99	    ;1
loop_2:		    ;98*10+9+1=990 cycles
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


