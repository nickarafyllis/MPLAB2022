.include "m328PBdef.inc"

main:
    ldi r16,0x55    ;initialize A
    ldi r17,0x43    ;initialize	B
    ldi r18,0x22    ;initialize C
    ldi r19,0x02    ;initialize D
    ldi r20,0x06    ;initialize loop counter
    rjmp loop	    ;jump to loop
 
;calculate result of function 0 and put it in register r21    
f_0:	    
    mov r21,r16
    com r21	    ;r21--> A'
    mov r22,r17
    com r22	    ;r22--> B'
    mov r23,r19	    ;r23--> D
    and r21,r22	    ;A' and B'
    and r22,r23	    ;B' and D'
    or  r21,r22	    ;(A' and B') or (B' and D)
    com r21	    ;r21-->((A' and B') or (B' and D))'
    ret

;calculate result of function 1 and put it in register r22  
f_1:
    mov r22,r16	    ;r22--> A
    mov r23,r18     ;r23--> C
    mov r24,r17     ;r24--> B
    mov r25,r19
    com r25	    ;r25--> D'
    or r22,r23      ;A or C
    or r24,r25      ;B or D'
    and r22,r24	    ;(A or C) and (B or D')
    ret
    
loop:
    rcall f_0	    ;r21 has the current value of f0
    rcall f_1	    ;r22 has the current value of f1
    subi r16,-2	    ;add 2 to A
    subi r17,-3	    ;add 3 to B
    subi r18,-4	    ;add 4 to C
    subi r19,-5	    ;add 5 to D
    dec r20	    ;decrease loop counter
    brne loop	    ;if loop counter is not zero continue loopping

