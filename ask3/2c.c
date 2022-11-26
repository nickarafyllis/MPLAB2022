#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

unsigned char duty_table[]={5,26,46,66,87,107,128,148,168,189,209,229,250}; //declare duty cycle table

int main() {
   //set TMR1A in fast PWM 8 bit mode with non-inverted output 
   //prescale=8
    TCCR1A = (1<<WGM10)| (1<<COM1A1);
    TCCR1B = (1<<WGM12)| (1<<CS11);
    
    DDRB |= 0b00111111; //set PB5-PB0 pins as output
    //DDRD = 0x00;    // ser PORTD as input
    
    int ptr = 6;  //initialize table pointer to 128 (50%)
    OCR1AL = duty_table[ptr];   //initialize duty cycle  to 50%
    
    while (1) {
        if((~PIND & 0x2)==2) {
            if(ptr<12){
                while((~PIND & 0x2)==2) {}   //debouncing filter
                OCR1AL = duty_table[++ptr]; 
            }
        }
           
        if((~PIND & 0x4)==4) {
            if(ptr>0){
                while((~PIND & 0x4)==4) {}   //debouncing filter
                OCR1AL = duty_table[--ptr]; 
            }
        }
    }      
}