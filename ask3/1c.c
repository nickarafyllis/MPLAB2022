#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


ISR (TIMER1_OVF_vect) { //TIMER1 ISR 
    if(PORTB & 4){      //if PB0-PB5 are on(refresh occurred)
        PORTB = 1;      //leave only PB0 on
        //3.5*15625=54687
        TCNT1H = 0x2A;  //65536-54687=10849(0x2A61)
        TCNT1L = 0x61;  //set timer for 3.5 second delay
    }
    else {              //else only PB0 is on
        PORTB = 0;      //turn it off
    }
}

ISR(INT1_vect) { // interrupt service routine 
    while(1){                       //Check EIFR
        EIFR = (1<<INTF1);
        _delay_ms(5);
        int f1 = EIFR && 0x2;
        if(f1 == 0)break;
    }
    if(PORTB & 1){   //Check PB0 led state
        PORTB = 63;     //if PB0 is on, set all leds of PORTB on for 0.5 seconds
        TCNT1H = 0xE1;  //65536-7812=57724(0xE17C)
        TCNT1L = 0x7C;  //set timer for 0.5 second delay
    }       
    else {
        PORTB = 1;      //else PB0 is off, set PB0 on
        TCNT1H = 0x0B;  //65536-62500=3036(0x0BDC)
        TCNT1L = 0XDC;  //set timer for 4 second delay
    }
}

int main() {
    //Interrupt on rising edge of INT1 pin
    EICRA=(1<<ISC11)|(1<<ISC10);
    //Enable the INT1 interrupt(PD3)
    EIMSK=(1<<INT1);
    //Enable interrupt of TCNT1 for timer1
    TIMSK1=(1<<TOIE1);
    TCCR1B=(1<<CS12)|(0<<CS11)|(1<<CS10); // CK/2024
    sei();        // Enable global interrupts
    DDRB=0xFF;    //Set PORTB as output

    while(1){
        if((PINC & 32) == 0){      //Check PC5
            while((PINC & 32) == 0) {}
            if(PORTB & 1){   //Check PB0 led state
                PORTB = 63;     //if PB0 is on, set all leds of PORTB on for 0.5 seconds
                //0.5*15625=7812
                TCNT1H = 0xE1;  //65536-7812=57724(0xE17C)
                TCNT1L = 0x7C;  //set timer for 0.5 second delay
            }       
            else{
                PORTB = 1;      //else PB0 is off, set PB0 on
                //4*15625=62500
                TCNT1H = 0x0B;  //65536-62500=3036(0x0BDC)
                TCNT1L = 0XDC;  //set timer for 4 second delay
            }            
        }
    }
    return 0;
}