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
// PCA9555 REGISTERS
char output;
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


uint8_t r24, r25;
uint8_t char_arr[9] = {'N', 'O', ' ', 'D', 'e', 'v', 'i', 'c', 'e'}; 

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

void write_2_nibbles(uint8_t byte){
    
    output = (byte & 0xF0) + (output & 0x0F); 
    PCA9555_0_write(REG_OUTPUT_0, output);      //sends 4 MSB
    output |= 0x08; // Enable Pulse
    PCA9555_0_write(REG_OUTPUT_0, output);
    output &= 0xF7;
    PCA9555_0_write(REG_OUTPUT_0, output);
    
    output = (byte & 0x0F)*16 + (output & 0x0F); 
    PCA9555_0_write(REG_OUTPUT_0, output);
    output |= 0x08; // Enable Pulse
    PCA9555_0_write(REG_OUTPUT_0, output);
    output &= 0xF7;
    PCA9555_0_write(REG_OUTPUT_0, output);
    return;
}

void lcd_command(uint8_t command){
    output |= 0b00000100;
    write_2_nibbles(command);
    _delay_us(100);
    return;
}

void lcd_data(uint8_t data){
    output &= 0b11111011; 
    write_2_nibbles(data);
    _delay_us(100);
}

void lcd_init(){
    _delay_ms(40); // Wait init of lcd
    int i;
    for(i = 0; i < 2; i++){
        output = 0x30;
        PCA9555_0_write(REG_OUTPUT_0, output);
        output |= 0x08;
        PCA9555_0_write(REG_OUTPUT_0, output);
        output &= 0xF7;
        PCA9555_0_write(REG_OUTPUT_0, output);
        _delay_us(100);
    }
	int temp = 0x28;		//chars of 5x8 pixels
	lcd_command(temp);	
	temp = 0x28;			//chars of 5x8 pixels
	lcd_command(temp);		
	temp = 0x0C;			//enable monitor
	lcd_command(temp);
	temp = 0x01;
	lcd_command(temp);		//clear monitor
     _delay_us(1530); 
    
	temp = 0x06;			//enable auto increment of counter address
	lcd_command(temp);
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
    //DDRD = 0xFF;            //set PORTD as output
    //lcd_init();             //init LCD
    r24 = 0;
    r25 = 0;
    while(1){
        lcd_command(1);       //clear display.
        get_temp();           //will save temperature on r24, r25 global variables
        if(r24 == 0 && r25 == 0x80){
            print_string();
        }
        else{
            if(r25 == 0xFF){            //if r25=0xFF, temperature is negative
                lcd_data('-');
                r24 = ~(r24 - 1);       //2's complement architecture for negative number
                uint8_t units = r24%10; 
                uint8_t decades = r24/10;
                lcd_data(decades);      //
                lcd_data(units);        //
                lcd_data(0b10110010);   //Print temperature values
                lcd_data('C');          //
            }
            else {                      //else temperature is positive
                uint8_t units = r24%10;     
                uint8_t decades = r24/10;
                lcd_data(decades);      //
                lcd_data(units);        //
                lcd_data(0b10110010);   //Print temperature values
                lcd_data('C');          //
            }
        }
        _delay_ms(2000);
    }
    return 0;
}
