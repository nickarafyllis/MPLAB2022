.include "m328PBdef.inc" ;ATmega328P microcontroller definitions

.def temp = r16 
.def led_bit = r17
.def ADC_L = r22
.def ADC_H = r23

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
	
	; REFSn[1:0]=01 => select Vref=5V,  MUXn[4:0]=0011 => select ADC3(pin PC3), 
	; ADLAR=0 => Right adjust the ADC result
	ldi temp, 0b01000011 ;
	sts ADMUX, temp
	
	ldi r24,1
	rcall lcd_command
	
	; ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	; ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
	ldi temp, 0b10000111
	sts ADCSRA, temp

	lds temp, ADCSRA ;
	ori temp, (1<<ADSC)|(1<<ADIE) ; Set ADSC flag of ADCSRA,ADIE to enable interrupt
	sts ADCSRA, temp ;
	sei				 ;enable interrupts

	
main:
	lds temp, ADCSRA ;
	ori temp, (1<<ADSC) ; Set ADSC flag of ADCSRA
	sts ADCSRA, temp ;
	ldi r24,low(10*16);
	ldi r25,high(10*16);
	rcall wait_msec	    ;delay 100 msec (wait till the interrupt occurs)
	
	andi ADC_H, 0b00000011  ;10bit value, keep only 2 lsb from high byte
	movw r25:r24, ADC_H:ADC_L  ;save the word in r24:r25
	lsl ADC_L		;multiply by 2
	rol ADC_H		;multiply by 2 and add carry 
	lsl ADC_L		;multiply by 2
	rol ADC_H		;multiply by 2 and add carry 
	add ADC_L,r24	;add initial word, same as multiplying by 5
	adc ADC_H,r25	
	ldi temp,10
divide:				;divide by 1024(10 shifts)
	lsr ADC_H	    ;shift right
	ror ADC_L	    ;shift right with carry from lsr
	dec temp	    ;
	brne divide	    ;repeat 10 times(10 bit value)

	ldi led_bit,1
convert:
	cpi ADC_L, 0
	breq check
	dec ADC_L
	lsl led_bit
	rjmp convert
	
check:
	;check if Vgas is more or less than danger level
	cpi led_bit, 1 
	brne more_than_70ppm    ;if less continue
	
	brtc led_on		    ;if t flag is not set only open led

	;else first send CLEAR message to display
	rcall lcd_init ;                    
	ldi r24, low(2)
	ldi r25, high(2) ;         2 msec
	rcall wait_msec
	ldi r24,'C'
	rcall lcd_data ;               byte                                   lcd
	ldi r24,'L'
	rcall lcd_data 
	ldi r24,'E'
	rcall lcd_data 
	ldi r24,'A'
	rcall lcd_data 
	ldi r24,'R'
	rcall lcd_data 

	clt ;clear t flag

led_on:
	out PORTB, led_bit; ;;;;;;;;;;
	ldi r24, low(100*16)
	ldi r25, high(100*16)
	rcall wait_msec
	rjmp main

more_than_70ppm:
	brts blink ;if t flag is already set go blinking
	;else first send message to lcd display
	
	set	    ;set t flag

	;send GAS DETECTED message to display    
	rcall lcd_init ; 
	ldi r24, low(2*16)
	ldi r25, high(2*16) ; 
	rcall wait_msec
	ldi r24,'G'
	rcall lcd_data ; 
	ldi r24,'A'
	rcall lcd_data ;
	ldi r24,'S'
	rcall lcd_data ;
	ldi r24,' '
	rcall lcd_data ;
	ldi r24,'D'
	rcall lcd_data ;
	ldi r24,'E'
	rcall lcd_data ;
	ldi r24,'T'
	rcall lcd_data ;
	ldi r24,'E'
	rcall lcd_data ;
	ldi r24,'C'
	rcall lcd_data ;
	ldi r24,'T'
	rcall lcd_data ;
	ldi r24,'E'
	rcall lcd_data ;
	ldi r24,'D'
	rcall lcd_data ;
blink:	
	out PORTB, led_bit    ;turn on led 
	ldi r24, low(50*16)
	ldi r25, high(50*16) 
	rcall wait_msec
	ldi temp, 0
	out PORTB, temp	    ;turn off led
	ldi r24, low(50*16)
	ldi r25, high(50*16) 
	rcall wait_msec
	
	rjmp main	    

adc_isp:
    push r24	    ;save r24,r25
    push r25
    lds ADC_L,ADCL ; Read ADC result(right adjusted)
    lds ADC_H,ADCH ;
    pop r25		;all done
    pop r24		;retrieve r24,r25
    reti
    
    	
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