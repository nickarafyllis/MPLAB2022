#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

void write_2_nibbles(char x){
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

void lcd_data(char x){
	PORTD |= 0b00000100;	//PD2 = 1
	write_2_nibbles(x);
	_delay_us(100);
	return;
}

void lcd_command(char x){
	
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
	return;
}

int main() {
    DDRB = 0xFF;       //PORTB as output
    DDRD = 0xFF;       //PORTD as output
    DDRC = 0x00;       //PORTC as input
    
    lcd_init();
    
    //init ADC
    //REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0010 => select ADC2(pin PC2)
	//ADLAR=0 => Right adjust the ADC result
    ADMUX = 0b01000011; ///
    //ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	//ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10000111;
    
    int led_bit;
    int flag = 0;
    
    while (1){
        ADCSRA |= (1<<ADSC);     //set ADSC flag of ADCSRA
        while(ADCSRA & 0b01000000);     //wait for ADSC to become 0
        int adc = ADC;  //read ADC
        if(adc>=0b1011011001){        //729
            led_bit=0b100000;
        }
        else if(adc>=0b11110011){   //243 
            led_bit=0b10000;
        }
        else if(adc>=0b1010001){   //81   
            led_bit=0b1000;
        }
        else if(adc>=0b11011){   //27  
            led_bit=0b100;
        }
        else if(adc>=0b1001){   //9   
            led_bit=0b10;
        }
        else if(adc>=0b111){   //3 
            led_bit=0b1;
        }
        else led_bit=0;     //0
        
        //check if Vgas is more or less than danger level
        if(adc>=0b11001101) {  //77ppm->205
            if(flag==0){
                flag=1;
                lcd_init();
                _delay_ms(15);
                //print_str('GAS DETECTED');
                lcd_data('G');
                lcd_data('A');
                lcd_data('S');
                lcd_data(' '); /////////
                lcd_data('D');
                lcd_data('E');
                lcd_data('T');
                lcd_data('E');
                lcd_data('C');
                lcd_data('T');
                lcd_data('E');
                lcd_data('D');
            }
            PORTB = led_bit; 
            _delay_ms(50);
            PORTB = 0; 
            _delay_ms(50);
        }
        else {
            if(flag==1){
                flag=0;
                lcd_init();
                _delay_ms(2);
                //print_str('CLEAR'); 
                lcd_data('C');
                lcd_data('L');
                lcd_data('E');
                lcd_data('A');
                lcd_data('R');
            }
            PORTB = led_bit; 
            _delay_ms(100);
        }
    }
    return 0;
}