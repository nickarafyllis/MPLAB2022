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

void print_num(char x){
    char num = '0'; 
    while ((x > 0) && (x < 10)){
        num += 1;
        x -= 1;
    }
    lcd_data(num);
    _delay_us(100);
    return;
}

int main() {
    //set TMR1A in fast PWM 8 bit mode with non-inverted output 
    int x;
    int changed = 0;
    char duty;
    int i;
    DDRB |= 0x02;       //
    DDRD = 0xFF;       //PORTD as output
    DDRC = 0x00;       //PORTC as input
    //init ADC
    //REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0001 => select ADC1(pin PC1),
	//ADLAR=0 => Right adjust the ADC result
    ADMUX = 0b01000001;
    //ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
	//ADIE=0 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
    ADCSRA = 0b10000111;
    //init timer1
    TCCR1A = (1<<WGM11)| (0<<WGM10)|(1<<COM1A1) ; 
    TCCR1B = (1<<WGM12)|(1<<WGM13)|(0<<CS12)|(1<<CS11)|(0<<CS10); //init prescaler to 8
    while (1) {
        if(changed){//if a button was pressed, enter here
            _delay_ms(15);
            ADCSRA |= (1<<ADSC);     //set ADSC flag of ADCSRA
            while(1){                //wait for ADSC to become 0
                unsigned int t = ADCSRA & 0b01000000;
                if (t == 0)
                    break;
            }
            int adc = ADC;
            adc *= 5;   //ADC*VREF        //isolate decimal points value
            int decimal =  adc & 1008;    //only bits 10 to 5 matter for 2 num precision.
            decimal = decimal >> 4;
            adc = adc / 1024;              //divide by 1024 to find rest of
            adc = adc & 0b00000111;        //the result
            
            //code that prints current duty cycle value and % char
            //change line of LCD screen code here
            int mask = 1;
            int dec = 0;
            for(i = 0; i < 6; i++){         //calculate decimal points value 
                int temp2 = decimal & mask;
                if(temp2){
                    switch(mask){
                        case 32:
                            dec +=50;
                            break;
                        case 16:
                            dec += 25;
                            break;
                        case 8:
                            dec += 12;
                            break;
                        case 4:
                            dec += 6;
                            break;
                        case 2:
                            dec += 3;
                            break;
                        case 1:
                            dec += 1;
                            break;
                    }
                }
                mask =  mask << 1;
            }
            lcd_init();
            _delay_ms(15);            
            lcd_data(duty);
            _delay_ms(2);
            lcd_data('0');
            _delay_ms(2);
            lcd_data('%');
            _delay_ms(2);
            lcd_command(0b11000000);
            _delay_ms(2);
            print_num(adc);//print adc (adc._ _)
            _delay_ms(2);
            lcd_data('.');   //print . 
            _delay_ms(2);
            int dec2 = dec % 10;          // _ . _ dec2 3.92v
            dec -= dec2;                  // _ . dec _
            dec /= 10;
            print_num(dec);              //print dec2 on screen
            _delay_ms(2);
            print_num(dec2);//print dec on screen
            _delay_ms(2);
            //done, reset changed
            changed = 0;
            while((PINB & 0b00111100) != 60)
                _delay_ms(5); 
        }
        x=~(PINB & 0b00111100);     //read PB2-PB5      
        if((x & 0x4)==4) {          //20% DC
            //while(((x=~(PINB & 0b00111100)) & 0x4) == 4);     //read PB2-PB5)
            OCR1A = 80; 
            ICR1 = 400;
            changed = 1;
            duty = '2';
        }
        else if((x & 0x8)==8) {     //40%DC
            //while(((x=~(PINB & 0b00111100)) & 0x8) == 8);     //read PB2-PB5)
            OCR1A = 160;
            ICR1 = 400;
            changed = 1;
            duty = '4';            
        }
        else if((x & 0x10)==16) {   //60%DC
            //while(((x=~(PINB & 0b00111100)) & 0x10) == 16);     //read PB2-PB5)
            OCR1A = 240;
            ICR1 = 400;
            changed = 1;
            duty = '6';
        }
        else if((x & 0x20)==32) {   //80%DC
            //while(((x=~(PINB & 0b00111100)) & 0x20) == 32);     //read PB2-PB5)
            OCR1A = 320;
            ICR1 = 400;
            changed = 1;
            duty = '8';
        }
        else {
            lcd_command(1);
            OCR1A=0;
            ICR1 = 400;
        }
    }  
    return 0;
}
