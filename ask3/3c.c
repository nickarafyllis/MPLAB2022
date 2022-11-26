#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned char x;

int main() {
    //set TMR1A in fast PWM 8 bit mode with non-inverted output 
    TCCR1A = (1<<WGM11)| (0<<WGM10)|(1<<COM1A1) ; 
    TCCR1B = (1<<WGM12)|(1<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10); //init prescaler to 8
    
    DDRB |= 0b00111111; //set PB5-PB0 pins as output
    //DDRD = 0x00;    // ser PORTD as input    
    
    while (1) {
        x=~PIND;      
        if((x & 0x1)==1) {
            OCR1A = 7999;
            ICR1 = 15999;
        }
        else if((x & 0x2)==2) {
            OCR1A = 3999;
            ICR1 = 7999;
        }
        else if((x & 0x4)==4) {
            OCR1A = 1999;
            ICR1 = 3999;
        }
        else if(x & 0x8) {
            OCR1A = 999;
            ICR1 = 1999; 
        }
        else {
            OCR1A=0;
            ICR1=0;
        }
    }      
}