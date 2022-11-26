#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int reset;  //reset check
int i=0;    //additional interrupt counter

ISR (INT1_vect) //External INT1 ISR
{   
    reset = 0;
    while(1){                       //Check EIFR
        EIFR = (1<<INTF1);
        _delay_ms(5);
        int f1 = EIFR && 0x2;
        if(f1 == 0)break;
    }
    sei();                          // Enable interrupts
    int j;
    if(i > 0) {                     //if interrupt variable is set, turn on all LEDs of portB for 0.5s 
        PORTB = 0xFF;               //adjust output
        for(j = 0; j < 500;j++){    //delay 0.5 s
            if(reset == 1) return;  //if reset == 1 then return, else keep delaying
            _delay_ms(1);
        }
    }
    i++;                            //increase interrupt variable
    PORTB = 0x01;                   //turn on LSB for 4 s
    for(j = 0; j < 4000;j++){       //delay 4 s
        if(reset == 1) return;      //if reset == 1 then return, else keep delaying
        else _delay_ms(1);
    }
    PORTB =0x00;                    //clear output
    i=0;                            //clear interrupt variable
    reset = 1;                      //set reset to 1 for potential previous interrupts and return.
}

int main() {
    //initialize reset to 0
    reset = 0;
    //Interrupt on rising edge of INT1 pin
    EICRA=(1<<ISC11)|(1<<ISC10);
    //Enable the INT1 interrupt(PD3)
    EIMSK=(1<<INT1);
    sei();                          // Enable global interrupts
    
    DDRB=0xFF;                      //Set PORTC as output
    PORTB=0x00;
    while(1){                       //infinite loop
        PORTB=0x00;                 //clear output
        reset = 0;                  //set reset to 0
    }  
}