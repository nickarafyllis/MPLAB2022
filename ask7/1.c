#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define PCA9555_0_ADDRESS 0x40 //A0=A1=A2=0 by hardware
#define TWI_READ 1 // reading from twi device
#define TWI_WRITE 0 // writing to twi device
#define SCL_CLOCK 100000L // twi clock in Hz
//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2
#define PD4 0x10
#define PD4_ 0xEF 

uint8_t r24, r25;

uint8_t one_wire_reset(){
    DDRD |= PD4;       //set PD4 as output
    //480 usec reset pulse
    PORTD &= PD4_;
    _delay_us(480);
    
    DDRD &= PD4_;      //set PD4 as input
    PORTD &= PD4_;     //disable pull-up
    
    //wait 100 usec for connected devices
    //to transmit the presence pulse
    _delay_us(100);
    
    uint8_t temp = PIND;      //read and save PORTD
    
    _delay_us(380);
    
    if(temp & PD4)
        return 0;      //if PD4 = 1 return 0 
    else 
        return 1;      //else return 1
}

uint8_t one_wire_receive_bit(){
    DDRD |= PD4;    //set PD4 as output
    
    PORTD &= PD4_;  //
    _delay_us(2);  //time slot 2 usec
    
    DDRD&= PD4_;    //set PD4 as input
    PORTD &= PD4_;  //disable pull-up
    
    _delay_us(10); //wait 10 usec
    
    uint8_t out = (PIND & PD4); //r24 = PD4
    
    _delay_us(49); //delay 49 usec to meet the standards
    
     return out;
} 

void one_wire_transmit_bit(uint8_t out_bit){
    DDRD |= PD4;        //set PD4 as output
    
    PORTD &= PD4_;      //
    _delay_us(2);       //time slot 2 usec
    
    PORTD |= out_bit;   //PD4 = out_bit value
    
    _delay_us(58);      //wait 58 usec for connected device to sample the line
    
    DDRD &= PD4_;       //set PD4 as input
    PORTD &= PD4_;      //disable pull-up
    
    _delay_us(1);        //recovery time 1 usec
    return;
}

uint8_t one_wire_receive_byte(){
    uint8_t mask = 1;
    uint8_t out = 0;
    for(int i=0; i<8; i++){ //; 8 repetitions
        if(one_wire_receive_bit()){
            out |= mask;
        }
        mask = mask << 1;
    }
    return out;
}

void one_wire_transmit_byte(uint8_t out_byte){
    int i;
    uint8_t mask = 1,temp;
    for(i = 0; i < 8; i++){
        temp = out_byte & mask;
        one_wire_transmit_bit(temp);
        mask = mask << 1;
    }
    return;
}

void get_temp(){
    if(one_wire_reset() == 0){      //init
        r25 = 0x80;
        r24 = 0x00;
        return;
    }

    one_wire_transmit_byte(0xCC);   //Send command 0xCC

    one_wire_transmit_byte(0x44);   //Send command 0x44
    PORTD &= PD4_;                  //set PD4 = 0
    while(one_wire_receive_bit());  //Wait for DS1820 to send bit with value 1

    if(one_wire_reset() == 0){      //init
        r25 = 0x80;
        r24 = 0x00;
        return;
    }

    one_wire_transmit_byte(0xCC);   //Send command 0xCC

    one_wire_transmit_byte(0xBE);   //Send command 0xBE
    r24 = one_wire_receive_byte();  //get first 8 bits(assuming we read 8 lsb first)
    r25 = one_wire_receive_byte();  //get rest 8 bits
    if(one_wire_reset() == 0){      //init
        r25 = 0x80;
        r24 = 0x00;
        return;
    }
}
int main() {      
    while(1){
        get_temp();
    }
    return 0;
}
