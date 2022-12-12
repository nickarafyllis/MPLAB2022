#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PD4 0x10        
#define PD4_ 0xEF 

uint8_t low, high;
uint8_t char_arr[9] = {'N', 'O', ' ', 'D', 'e', 'v', 'i', 'c', 'e'}; 


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
    
    PORTD |= (out_bit << 4) ;   //PD4 = out_bit value
    
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
        if(out_byte & mask)
            temp = 1;
        else 
            temp = 0;
        one_wire_transmit_bit(temp);
        mask = mask << 1;
    }
    return;
}

void get_temp(){
    if(one_wire_reset() == 0){      //init
        low = 0x00;
        high = 0x80;
        return;
    }

    one_wire_transmit_byte(0xCC);   //Send command 0xCC

    one_wire_transmit_byte(0x44);   //Send command 0x44
    PORTD &= PD4_;                  //set PD4 = 0
    while(one_wire_receive_bit());  //Wait for DS1820 to send bit with value 1

    if(one_wire_reset() == 0){      //init
        high = 0x80;
        low = 0x00;
        return;
    }

    one_wire_transmit_byte(0xCC);   //Send command 0xCC

    one_wire_transmit_byte(0xBE);   //Send command 0xBE
    low = one_wire_receive_byte();  //get first 8 bits(assuming we read 8 lsb first)
    high = one_wire_receive_byte();  //get rest 8 bits
    if(one_wire_reset() == 0){      //init
        high = 0x80;
        low = 0x00;
        return;
    }
}


void write_2_nibbles(unsigned char x){
	int temp = x;		//sends 4 MSB
	int d = PIND;
	d = d & 0x0F;
	temp = d & 0xF0;
	temp += d;
	PORTD = x;
	PORTD |= 0b00001000;	//enable pulse
	PORTD &= 0b11110111;
	temp = x;
	temp = temp << 4;	//switch 4 LSB to 4 MSB
	temp += d;			//send
	PORTD = temp;
	PORTD |= 0b00001000;	//enable pulse
	PORTD &= 0b11110111;
	return;
}

void lcd_data(unsigned char x){
	PORTD |= 0b00000100;	//PD2 = 1
	write_2_nibbles(x);
	_delay_us(100);
	return;
}

void lcd_command(unsigned char x){
	
	PORTD &= 0b11111011;	//PD2 = 0
	write_2_nibbles(x);
	_delay_us(100);
	return;
}

void lcd_init(){
	_delay_ms(40);				//wait init of lcd
	int i;		
	for(i = 0; i < 2; i++){		//make sure to switch to 8 bit mode
		int temp = 0x30;		//2 or 3 loops to work
		PORTD = temp;
		PORTD |= 0b00001000;	//enable pulse
		PORTD &= 0b11110111;
		_delay_us(39);
	}
	int temp = 0x20;		//switch to 4 bit mode
	PORTD = temp;
	PORTD |= 0b00001000;	//enable pulse
	PORTD &= 0b11110111;
	_delay_us(39);
	temp = 0x28;			//chars of 5x8 pixels
	lcd_command(temp);		
	temp = 0x0C;			//enable monitor
	lcd_command(temp);
	temp = 0x01;
	lcd_command(temp);		//clear monitor
	_delay_us(1530);
	temp = 0x06;			//enable auto increment of counter address
	lcd_command(temp);
	//return;
}

void print_string(){
    int i = 0;
    for(i = 0; i < 9; i++){
        uint8_t temp = char_arr[i];
        lcd_data(temp);
    }
    return;
}

int main() {
    DDRD = 0xFF;            //set PORTD as output
    
    
    lcd_init();             //init LCD
    low = 0;
    high = 0;
    while(1){
        lcd_command(1);       //clear display. 
        get_temp();           //will save temperature on low, high global variables
        _delay_ms(3);
        if(low == 0 && high == 0x80){       
            DDRD = 0xFF;            //set PORTD as output
            print_string();
        }
        else{
            DDRD = 0xFF;            //set PORTD as output
            if(high == 0b11111000){            //if high=0xFF, temperature is negative
                lcd_data('-');
                uint16_t both = (high & 0b0111);                
                both = (both  << 8) + low;
                both = ~(both-1);       //2's complement architecture for negative number
                both = both*0.0625;
                uint8_t units = both%10; 
                uint8_t decades = both/10;
                _delay_ms(1);
                lcd_data('0'+decades);      // cast to ascii
                lcd_data('0'+units);        // cast to ascii
                lcd_data(0b10110000);   //Print temperature values
                _delay_ms(1);                
                lcd_data('C');          //
            }
            else {                      //else temperature is positive
                uint16_t both = (high & 0b0111);                
                both = (both  << 8) + low;               
                both = both*0.0625;
                uint8_t units = both%10; 
                uint8_t decades = both/10;
                _delay_ms(1);
                lcd_data('0'+decades);      // cast to ascii
                _delay_ms(1);
                lcd_data('0'+units);        // cast to ascii
                //extended ascii character not available
                /*
                _delay_ms(3);
                lcd_data('°');       //Print degree symbol 
                */
                _delay_ms(1);
                lcd_data('C');      // Print Celsius symbol
            }
        }
        _delay_ms(2000);
    }
    return 0;
}
