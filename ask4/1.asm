.include "m328PBdef.inc" ;ATmega328P microcontroller definitions

.def temp = r16
.def ADC_L = r22
.def ADC_H = r23
.def counter = r21

.org 0x00
jmp reset
.org 0x2A ;ADC Conversion Complete Interrupt
jmp adc_isp

reset:
	ldi temp, high(RAMEND)		;Initialise Stack Pointer
	out SPH,temp
	ldi temp, low(RAMEND)
	out SPL,temp
	
	ldi temp, 0xFF
	out DDRB, temp ;Set PORTB as output
	ldi temp, 0xFF
	out DDRD, temp ;Set PORTD as output
	ldi temp, 0x00
	out DDRC, temp ;Set PORTC as input
	
	; REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0010 => select ADC2(pin PC2),
	; ADLAR=0 => Right adjust the ADC result
	ldi temp, 0b01000010 ;
	sts ADMUX, temp
	; ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	; ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
	ldi temp, 0b10000111
	sts ADCSRA, temp
	rcall lcd_init
	lds temp, ADCSRA ;
	ori temp, (1<<ADSC)|(1<<ADIE) ; Set ADSC flag of ADCSRA,ADIE to enable interrupt
	sts ADCSRA, temp ;
	sei				 ;enable interrupts
	ldi counter,1;			 ;init counter to 1
main:	
	ldi r24,1;
	rcall lcd_command ;
	lds temp, ADCSRA ;
	ori temp, (1<<ADSC) ; Set ADSC flag of ADCSRA
	sts ADCSRA, temp ;
	out PORTB, counter;
	ldi r24,low(1000*16);
	ldi r25,high(1000*16);
	rcall wait_msec	    ;delay 1 sec
	inc counter	    ;increment counter
	rjmp main	    

adc_isp:
    push r24	    ;save r24,r25
    push r25
    lds ADC_L,ADCL ; Read ADC result(right adjusted)
    lds ADC_H,ADCH ;
    andi ADC_H, 0b00000011  ;10bit value, keep only 2 lsb from high byte
    movw r24, r22	;save the word in r24:r25
    lsl ADC_L		;multiply by 2
    rol ADC_H		
    lsl ADC_L		;multiply by 2
    rol ADC_H
    add ADC_L,r24	;add initial word, same as multiplying by 5
    adc ADC_H,r25	
    ldi temp,10
divide:				;divide by 1024(10 shifts)
    lsr ADC_H	    ;shift right
    ror ADC_L	    ;shift right with carry from lsr
    ror r17	    ;r17 will contain decimal points in binary form
    dec temp	    ;
    brne divide	    ;repeat 10 times(10 bit value)
    ;calculate decimal point with 2 digit precision
    clr r18			;store it on r18
    lsl r17
    brcc skip1
    subi r18,-50		;0,5
skip1:
    lsl r17
    brcc skip2
    subi r18,-25		;0,25
skip2:
    lsl r17
    brcc skip3
    subi r18,-12		;0,125
skip3:
    lsl r17
    brcc skip4
    subi r18,-6		;0,0625
skip4:
    lsl r17
    brcc skip5
    subi r18,-3		;0,03125
skip5:
    lsl r17
    brcc skip6
    subi r18,-1		;0,015625
skip6:
    ldi r20, 0b00110000	;initialize output to '0'
    clr r16		;init 1st decimal digit to 0
    clr r17		;init 2nd decimal digit to 0

find1:
    cpi r18,0
    breq loop1		;if r17 = 0 we are done
    dec r18		;decrement r18
    inc r17		;increase 2nd decimal point
    cpi r17,10		;if we havent reached 10
    brne find1		;jump to find1
    inc r16		;else increase 1st decimal
    clr r17		;clear 2nd
    rjmp find1		;continue looping
    ldi r20, '0'	;initialize output to '0'
loop1:			;adjust output for 1st digit
    cpi r22,0
    breq print1
    inc r20
    dec r22
    rjmp loop1		;r17-->2nd decimal,r16-->1st decimal,r22-->1st digit
print1:
    rcall send		;output 1st digit
print_comma:
    ldi r20, '.'	;initialize output to '.'
    rcall send		;output comma
    ldi r20, '0'	;initialize output to '0'    
loop2:			;adjust output for 1st decimal digit
    cpi r16,0
    breq print2
    inc r20
    dec r16
    rjmp loop2
print2:
    rcall send		;output 1st decimal digit
    ldi r20, '0'	;initialize output to '0'
loop3:			;adjust output for 2nd decimal digit
    cpi r22,0
    breq print3
    inc r20
    dec r22
    rjmp loop3
print3:
    rcall send		;output 1st digit
    pop r25		;all done
    pop r24		;retrieve r24,r25
    reti


send:
	ldi r24, low(2*16)
	ldi r25, high(2*16) ;         2 msec
	rcall wait_msec
	mov r24,r20
	rcall lcd_data ;               byte                                   lcd
	ret    
	
write_2_nibbles:
    push r24 ;            4 MSB
    in r25 ,PIND ;                4 LSB                     
    andi r25 ,0x0f ;                                                     
    andi r24 ,0xf0 ;                  4 MSB    
    add r24 ,r25 ;                                 4 LSB
    out PORTD ,r24 ;                        
    sbi PORTD ,3 ;                      Enable                PD3
    cbi PORTD ,3 ; PD3=1          PD3=0
    pop r24 ;            4 LSB.              byte.
    swap r24 ;                  4 MSB       4 LSB
    andi r24 ,0xf0 ;                                    
    add r24 ,r25
    out PORTD ,r24
    sbi PORTD ,3 ;             Enable
    cbi PORTD ,3
    ret

lcd_init:
ldi r24 ,low(40*16) ;                     lcd                 
ldi r25 ,high(40*16) ;                                        .
rcall wait_msec ;         40 msec                          .
ldi r24 ,0x30 ;                     8 bit mode
out PORTD ,r24 ;                                       
sbi PORTD ,3 ;                                      
cbi PORTD ,3 ;           ,                                
ldi r24 ,39
ldi r25 ,0 ;                                        8-bit mode
rcall wait_usec ;                     ,                                   
;         4 bit                          8 bit
ldi r24 ,0x30
out PORTD ,r24
sbi PORTD ,3
cbi PORTD ,3
ldi r24 ,39
ldi r25 ,0
rcall wait_usec
ldi r24 ,0x20 ;           4-bit mode
out PORTD ,r24
sbi PORTD ,3
cbi PORTD ,3
ldi r24 ,39
ldi r25 ,0
rcall wait_usec
ldi r24 ,0x28 ;                             5x8         
rcall lcd_command ;                                    
ldi r24 ,0x0c ;                        ,                     
rcall lcd_command
ldi r24 ,0x01 ;                      
rcall lcd_command
ldi r24 ,low(1530)
ldi r25 ,high(1530)
rcall wait_usec
ldi r24 ,0x06 ;                                     1               
rcall lcd_command ;                                                    
;                                                  
ret    
    
lcd_data:
sbi PORTD ,2 ;                                  (PD2=1)
rcall write_2_nibbles ;              byte
ldi r24 ,143 ;         43 sec                            
ldi r25 ,0 ;                                   lcd
rcall wait_usec
ret

lcd_command:
cbi PORTD ,2 ;                                (PD2=1)
rcall write_2_nibbles ;                                  39 sec
ldi r24 ,139 ;                                                          lcd.
ldi r25 ,0 ;    .:                     , clear display     return home,
rcall wait_usec ;                                                   .
ret

wait_usec:
sbiw r24 ,1 ; 2       
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
nop ; 1        
brne wait_usec ; 1   2        
ret ; 4        

wait_msec:
loop_1:
    ldi r26, 99	;1
loop_2:		    ;10*99=990 cycles
    rcall wait4	    ;3+4 = 7
    dec r26	    ;1
    brne loop_2	    ;2 
    sbiw r25:r24, 1 ;2 cycles
    brne stall	    ;if value > 0 then 2 cycle, if value = 1 then 2 cycles.
    ret		    ;4+3
stall:		    ;we need 6 more cycles to reach 1000 if the branch is taken 
    nop		    ;1
    nop		    ;1
    nop		    ;1
    nop		    ;1
    rjmp loop_1	    ;2 ,total 6 cycles
    
wait4:
    ret		    ;4 cycles


