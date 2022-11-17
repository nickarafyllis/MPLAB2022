.include "m328PBdef.inc" ;ATmega328P microcontroller definitions

.def temp = r16
.def ADC_L = r21
.def ADC_H = r22

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
	ldi temp, 0x00
	out DDRC, temp ;Set PORTC as input
	; REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0000 => select ADC0(pin PC0),
	; ADLAR=1 => Left adjust the ADC result
	ldi temp, 0b01100000 ;
	sts ADMUX, temp
	; ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	; ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
	ldi temp, 0b10000111
	sts ADCSRA, temp
	
Start_conv:
	lds temp, ADCSRA ;
	ori temp, (1<<ADSC)|(1<<ADIE) ; Set ADSC flag of ADCSRA,ADIE to enable interrupt
	sts ADCSRA, temp ;
	sei				 ;enable interrupts

adc_isp:
	reti
