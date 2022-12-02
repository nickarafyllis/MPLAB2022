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
// PCA9555 REGISTERS
typedef enum {
REG_INPUT_0 = 0,
REG_INPUT_1 = 1,
REG_OUTPUT_0 = 2,
REG_OUTPUT_1 = 3,
REG_POLARITY_INV_0 = 4,
REG_POLARITY_INV_1 = 5,
REG_CONFIGURATION_0 = 6,
REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;
//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10
//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58
#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)
//initialize TWI clock
void twi_init(void)
{
    TWSR0 = 0; // PRESCALER_VALUE=1
    TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}
// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void)
{
    TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
    while(!(TWCR0 & (1<<TWINT)));
    return TWDR0;
}

//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void)
{
    TWCR0 = (1<<TWINT) | (1<<TWEN);
    while(!(TWCR0 & (1<<TWINT)));
    return TWDR0;
}

// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address)
{
    uint8_t twi_status;
        // send START condition
        TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
        // wait until transmission completed
        while(!(TWCR0 & (1<<TWINT)));
        // check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
        // send device address
        TWDR0 = address;
        TWCR0 = (1<<TWINT) | (1<<TWEN);
        // wail until transmission completed and ACK/NACK has been received
        while(!(TWCR0 & (1<<TWINT)));
        // check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) )
    {
    return 1;
    }
        return 0;
}
// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address)
{
    uint8_t twi_status;
    while ( 1 )
    {
        // send START condition
        TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
        // wait until transmission completed
        while(!(TWCR0 & (1<<TWINT)));
        // check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
        // send device address
        TWDR0 = address;
        TWCR0 = (1<<TWINT) | (1<<TWEN);
        // wail until transmission completed
        while(!(TWCR0 & (1<<TWINT)));
        // check value of TWI Status Register.
        twi_status = TW_STATUS & 0xF8;
        if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) )
        {
            /* device busy, send stop condition to terminate write operation */
            TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
            // wait until stop condition is executed and bus released
            while(TWCR0 & (1<<TWSTO));
            continue;
        }
        break;
    }
}

// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data )
{
    // send data to the previously addressed device
    TWDR0 = data;
    TWCR0 = (1<<TWINT) | (1<<TWEN);
    // wait until transmission completed
    while(!(TWCR0 & (1<<TWINT)));
    if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
    return 0;
}

// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address)
{
    return twi_start( address );
}
// Terminates the data transfer and releases the twi bus
void twi_stop(void)
{
    // send stop condition
    TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    // wait until stop condition is executed and bus released
    while(TWCR0 & (1<<TWSTO));
}
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value)
{
    twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
    twi_write(reg);
    twi_write(value);
    twi_stop();
}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg)
{
uint8_t ret_val;
    twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
    twi_write(reg);
    twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
    ret_val = twi_readNak();
    twi_stop();
    return ret_val;
}

uint8_t scan_row(uint8_t row){
    uint8_t ret_val;
    if((row > 5) || (row < 1))                  //error check
        return -1;
    //might not be needed
    PCA9555_0_write(REG_CONFIGURATION_1, 0xF0); //Set IO1_0 as output, IO1_4-7 as input
    //activate desired row , 1 - 4.
    uint8_t x = 1;
    x = ~(x << row);
    PCA9555_0_write(REG_OUTPUT_1, x);
    //check if desired row's buttons are pressed.
    _delay_ms(1);
    ret_val = PCA9555_0_read(REG_INPUT_1);
    ret_val = ret_val & 0xF0;
    ret_val = ret_val >> 4;
    return ret_val; 
    
}

uint8_t* scan_keypad(){
    uint8_t i, j, x, mask;
    static uint8_t ret_arr[16] = {0};
    for(i = 0; i < 4; i++){
        x = scan_row(i+1);      //read current row
        mask = 1;               //init mask
        for(j = 0; j < 4; j++){ //save which buttons of current row are pressed
            if(x & mask)
                ret_arr[i*4+j] = 1;  //if button is pressed, adjust button table
            mask = mask << 1;   //adjust mask for next button check
        }
    }
    return ret_arr;
}

uint8_t* scan_keypad_rising_edge(){
    //can we return 4x4 array to pointer???
    uint8_t i; 
    static uint8_t ret_arr[16] = {0};
    uint8_t *p1, *p2; 
    p1 = scan_keypad();
    _delay_ms(15);
    p2 = scan_keypad();
    for(i = 0; i < 16; i++){
        if(p1[i] != p2[i])
            ret_arr[i] = 1;
    }
    return ret_arr;
}

uint8_t keypad_to_ascii(){
    uint8_t i, found = -1;
    uint8_t char_arr[] = {'*','0', '#', 'D', '7', '8', '9', 'C', '4', '5', '6', 'B', '1', '2', '3', 'A'};
    uint8_t* p1 = scan_keypad_rising_edge();
    for(i = 0; i < 16; i++){
        if(p1[i] == 1){
            found = i;               //adjust found value
            break;
        }
    }
    //assumming that i value is 16 on the end of for loop if no button was pressed
    if(i != 16)
        return char_arr[found];
    else
        return 0;
}

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

int main(){
    DDRD = 0xFF;            //set PORTD as output
    lcd_init();             //init LCD
    _delay_ms(5);
    twi_init();             //init twi
    PCA9555_0_write(REG_CONFIGURATION_1, 0xF0); //Set IO1_0-3 as output, IO1_4-7 as input
    _delay_ms(5);
    uint8_t t;
    while(1){
        t = keypad_to_ascii();
        if(t != 0)  // if a button was pressed , change character displayed no the LCD screen
            lcd_command(1);
            _delay_ms(5);
            lcd_data(t);
    }
    return 0;
}