#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#define PCA9555_0_ADDRESS 0x40 // A0=A1=A2=0 by hardware
#define TWI_READ 1             // reading from twi device
#define TWI_WRITE 0            // writing to twi device
#define SCL_CLOCK 100000L      // twi clock in Hz
// Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU / SCL_CLOCK) - 16) / 2

uint8_t ret_arr[16] = {0};
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
// initialize TWI clock
void twi_init(void) {
  TWSR0 = 0;           // PRESCALER_VALUE=1
  TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}
// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void) {
  TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
  while (!(TWCR0 & (1 << TWINT)))
    ;
  return TWDR0;
}

// Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void) {
  TWCR0 = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR0 & (1 << TWINT)))
    ;
  return TWDR0;
}

// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address) {
  uint8_t twi_status;
  // send START condition
  TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
  // wait until transmission completed
  while (!(TWCR0 & (1 << TWINT)))
    ;
  // check value of TWI Status Register.
  twi_status = TW_STATUS & 0xF8;
  if ((twi_status != TW_START) && (twi_status != TW_REP_START))
    return 1;
  // send device address
  TWDR0 = address;
  TWCR0 = (1 << TWINT) | (1 << TWEN);
  // wail until transmission completed and ACK/NACK has been received
  while (!(TWCR0 & (1 << TWINT)))
    ;
  // check value of TWI Status Register.
  twi_status = TW_STATUS & 0xF8;
  if ((twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK)) {
    return 1;
  }
  return 0;
}
// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address) {
  uint8_t twi_status;
  while (1) {
    // send START condition
    TWCR0 = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    // wait until transmission completed
    while (!(TWCR0 & (1 << TWINT)))
      ;
    // check value of TWI Status Register.
    twi_status = TW_STATUS & 0xF8;
    if ((twi_status != TW_START) && (twi_status != TW_REP_START))
      continue;
    // send device address
    TWDR0 = address;
    TWCR0 = (1 << TWINT) | (1 << TWEN);
    // wail until transmission completed
    while (!(TWCR0 & (1 << TWINT)))
      ;
    // check value of TWI Status Register.
    twi_status = TW_STATUS & 0xF8;
    if ((twi_status == TW_MT_SLA_NACK) || (twi_status == TW_MR_DATA_NACK)) {
      /* device busy, send stop condition to terminate write operation */
      TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
      // wait until stop condition is executed and bus released
      while (TWCR0 & (1 << TWSTO))
        ;
      continue;
    }
    break;
  }
}

// Send one byte to twi device, Return 0 if write successful or 1 if write
// failed
unsigned char twi_write(unsigned char data) {
  // send data to the previously addressed device
  TWDR0 = data;
  TWCR0 = (1 << TWINT) | (1 << TWEN);
  // wait until transmission completed
  while (!(TWCR0 & (1 << TWINT)))
    ;
  if ((TW_STATUS & 0xF8) != TW_MT_DATA_ACK)
    return 1;
  return 0;
}

// Send repeated start condition, address, transfer direction
// Return: 0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address) {
  return twi_start(address);
}
// Terminates the data transfer and releases the twi bus
void twi_stop(void) {
  // send stop condition
  TWCR0 = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  // wait until stop condition is executed and bus released
  while (TWCR0 & (1 << TWSTO))
    ;
}
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value) {
  twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
  twi_write(reg);
  twi_write(value);
  twi_stop();
}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg) {
  uint8_t ret_val;
  twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
  twi_write(reg);
  twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
  ret_val = twi_readNak();
  twi_stop();
  return ret_val;
}

uint8_t scan_row(uint8_t row) {
  uint8_t ret_val;
  if ((row > 5) || (row < 1)) // error check
    return -1;
  // might not be needed
  PCA9555_0_write(REG_CONFIGURATION_1,
                  0xF0); // Set IO1_0 as output, IO1_4-7 as input
  // activate desired row , 1 - 4.
  uint8_t x = 1;
  x = ~(x << (row - 1));
  PCA9555_0_write(REG_OUTPUT_1, x);
  // check if desired row's buttons are pressed.
  _delay_ms(1);
  ret_val = PCA9555_0_read(REG_INPUT_1);
  ret_val = ~(ret_val & 0xF0);
  ret_val = ret_val >> 4;
  return ret_val;
}

uint8_t *scan_keypad() {
  int i, j;
  for (i = 0; i < 16; i++) {
    ret_arr[i] = 0;
  }
  uint8_t x, mask;
  for (i = 0; i < 4; i++) {
    x = scan_row(i + 1);      // read current row
    mask = 1;                 // init mask
    for (j = 0; j < 4; j++) { // save which buttons of current row are pressed
      if (x & mask)
        ret_arr[i * 4 + j] = 1; // if button is pressed, adjust button table
      mask = mask << 1;         // adjust mask for next button check
    }
  }
  return ret_arr;
}

uint8_t *scan_keypad_rising_edge() {
  // can we return 4x4 array to pointer???
  int i;
  uint8_t p1[16] = {0};
  uint8_t p2[16] = {0};
  uint8_t *temp = scan_keypad();
  for (i = 0; i < 16; i++) {
    p1[i] = temp[i];
  }
  _delay_ms(15);
  temp = scan_keypad();
  for (i = 0; i < 16; i++) {
    p2[i] = temp[i];
  }
  for (i = 0; i < 16; i++) {
    ret_arr[i] = 0;
  }
  for (i = 0; i < 16; i++) {
    if (p1[i] < p2[i])
      ret_arr[i] = 1;
  }
  return ret_arr;
}

uint8_t keypad_to_ascii() {
  int i, found = 0;
  uint8_t char_arr[] = {'*', '0', '#', 'D', '7', '8', '9', 'C',
                        '4', '5', '6', 'B', '1', '2', '3', 'A'};
  uint8_t *p1 = scan_keypad_rising_edge();
  for (i = 0; i < 16; i++) {
    if (p1[i] == 1) {
      found = i; // adjust found value
      break;
    }
  }
  // assumming that i value is 16 on the end of for loop if no button was
  // pressed
  if (i != 16)
    return char_arr[found];
  else
    return 0;
}

#define BUFFER_SIZE 128

#define PD4 0x10
#define PD4_ 0xEF

uint8_t low, high;

uint8_t one_wire_reset() {
  DDRD |= PD4; // set PD4 as output
  // 480 usec reset pulse
  PORTD &= PD4_;
  _delay_us(480);

  DDRD &= PD4_;  // set PD4 as input
  PORTD &= PD4_; // disable pull-up

  // wait 100 usec for connected devices
  // to transmit the presence pulse
  _delay_us(100);

  uint8_t temp = PIND; // read and save PORTD

  _delay_us(380);

  if (temp & PD4)
    return 0; // if PD4 = 1 return 0
  else
    return 1; // else return 1
}

uint8_t one_wire_receive_bit() {
  DDRD |= PD4; // set PD4 as output

  PORTD &= PD4_; //
  _delay_us(2);  // time slot 2 usec

  DDRD &= PD4_;  // set PD4 as input
  PORTD &= PD4_; // disable pull-up

  _delay_us(10); // wait 10 usec

  uint8_t out = (PIND & PD4); // r24 = PD4

  _delay_us(49); // delay 49 usec to meet the standards

  return out;
}

void one_wire_transmit_bit(uint8_t out_bit) {
  DDRD |= PD4; // set PD4 as output

  PORTD &= PD4_; //
  _delay_us(2);  // time slot 2 usec

  PORTD |= (out_bit << 4); // PD4 = out_bit value

  _delay_us(58); // wait 58 usec for connected device to sample the line

  DDRD &= PD4_;  // set PD4 as input
  PORTD &= PD4_; // disable pull-up

  _delay_us(1); // recovery time 1 usec
  return;
}

uint8_t one_wire_receive_byte() {
  uint8_t mask = 1;
  uint8_t out = 0;
  for (int i = 0; i < 8; i++) { //; 8 repetitions
    if (one_wire_receive_bit()) {
      out |= mask;
    }
    mask = mask << 1;
  }
  return out;
}

void one_wire_transmit_byte(uint8_t out_byte) {
  int i;
  uint8_t mask = 1, temp;
  for (i = 0; i < 8; i++) {
    if (out_byte & mask)
      temp = 1;
    else
      temp = 0;
    one_wire_transmit_bit(temp);
    mask = mask << 1;
  }
  return;
}

void get_temp() {
  if (one_wire_reset() == 0) { // init
    low = 0x00;
    high = 0x80;
    return;
  }

  one_wire_transmit_byte(0xCC); // Send command 0xCC

  one_wire_transmit_byte(0x44); // Send command 0x44
  PORTD &= PD4_;                // set PD4 = 0
  while (one_wire_receive_bit())
    ; // Wait for DS1820 to send bit with value 1

  if (one_wire_reset() == 0) { // init
    high = 0x80;
    low = 0x00;
    return;
  }

  one_wire_transmit_byte(0xCC); // Send command 0xCC

  one_wire_transmit_byte(0xBE); // Send command 0xBE
  low = one_wire_receive_byte(); // get first 8 bits(assuming we read 8 lsb first)
  high = one_wire_receive_byte(); // get rest 8 bits
  if (one_wire_reset() == 0) {    // init
    high = 0x80;
    low = 0x00;
    return;
  }
}

void write_2_nibbles(unsigned char x) {
  int temp = x; // sends 4 MSB
  int d = PIND;
  d = d & 0x0F;
  temp = d & 0xF0;
  temp += d;
  PORTD = x;
  PORTD |= 0b00001000; // enable pulse
  PORTD &= 0b11110111;
  temp = x;
  temp = temp << 4; // switch 4 LSB to 4 MSB
  temp += d;        // send
  PORTD = temp;
  PORTD |= 0b00001000; // enable pulse
  PORTD &= 0b11110111;
  return;
}

void lcd_data(unsigned char x) {
  PORTD |= 0b00000100; // PD2 = 1
  write_2_nibbles(x);
  _delay_us(100);
  return;
}

void lcd_command(unsigned char x) {

  PORTD &= 0b11111011; // PD2 = 0
  write_2_nibbles(x);
  _delay_us(100);
  return;
}

void lcd_init() {
  _delay_ms(40); // wait init of lcd
  int i;
  for (i = 0; i < 2; i++) { // make sure to switch to 8 bit mode
    int temp = 0x30;        // 2 or 3 loops to work
    PORTD = temp;
    PORTD |= 0b00001000; // enable pulse
    PORTD &= 0b11110111;
    _delay_us(39);
  }
  int temp = 0x20; // switch to 4 bit mode
  PORTD = temp;
  PORTD |= 0b00001000; // enable pulse
  PORTD &= 0b11110111;
  _delay_us(39);
  temp = 0x28; // chars of 5x8 pixels
  lcd_command(temp);
  temp = 0x0C; // enable monitor
  lcd_command(temp);
  temp = 0x01;
  lcd_command(temp); // clear monitor
  _delay_us(1530);
  temp = 0x06; // enable auto increment of counter address
  lcd_command(temp);
  // return;
}

/* Routine: usart_init
Description:
This routine initializes the
usart as shown below.
------- INITIALIZATIONS -------
Baud rate: 9600 (Fck= 8MH)
Asynchronous mode
Transmitter on
Reciever on
Communication parameters: 8 Data ,1 Stop, no Parity
--------------------------------
parameters: ubrr to control the BAUD.
return value: None.*/
void usart_init(unsigned int ubrr) {
  UCSR0A = 0;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0C = (3 << UCSZ00);
  return;
}
/* Routine: usart_transmit
Description:
This routine sends a byte of data
using usart.
parameters:
data: the byte to be transmitted
return value: None. */
void usart_transmit(uint8_t data) {
  while (!(UCSR0A & (1 << UDRE0)))
    ;
  UDR0 = data;
}

/* Routine: usart_receive
Description:
This routine receives a byte of data
from usart.
parameters: None.
return value: the received byte */
uint8_t usart_receive() {
  while (!(UCSR0A & (1 << RXC0)))
    ;
  return UDR0;
}

void writeMessageToLcd(const char *message) {
  lcd_command(0x01); // Clear the LCD screen
  lcd_command(0x80); // Set the cursor to the first character of the first line
  _delay_ms(10);
  for (int i = 0; i < strlen(message); i++) {
    lcd_data(message[i]);
  }
}

void transmitStringOverUart(const char *message) {
  for (int i = 0; i < strlen(message); i++) {
    usart_transmit(message[i]);
  }
}

char *readMessageFromUart(char *buffer, int bufferSize) {
  int messageIndex = 0;

  // Loop until we reach the end of the message (indicated by a newline
  // character)
  while (1) {
    // Read a character from the UART interface
    char c = usart_receive();

    // If we've reached the end of the message, break out of the loop
    if (c == '\n') {
      break;
    }

    // Otherwise, add the character to the message buffer
    buffer[messageIndex] = c;
    messageIndex++;

    // If the message buffer is full, break out of the loop
    if (messageIndex == bufferSize) {
      break;
    }
  }

  // Add a null terminator to the end of the message
  buffer[messageIndex] = '\0';

  return buffer;
}




int main() {
  DDRD = 0xFF; // set PORTD as output
  DDRC = 0x00; // PORTC as input
  // init ADC
  // REFSn[1:0]=01 => select Vref=5V, MUXn[4:0]=0000 => select     ADC0(pin
  // PC0), ADLAR=0 => Right adjust the ADC result
  ADMUX = 0b01000000;
  // ADEN=1 => ADC Enable, ADCS=0 => No Conversion,
  // ADIE=1 => disable adc interrupt, ADPS[2:0]=111 => fADC=16MHz/128=125KHz
  ADCSRA = 0b10000111;

  lcd_init(); // init LCD
  _delay_us(50);

  // Initialize the UART interface with a baud rate of 9600
  usart_init(103);
  _delay_us(50);

  twi_init(); // init twi
  PCA9555_0_write(REG_CONFIGURATION_1,
                  0xF0); // Set IO1_0-3 as output, IO1_4-7 as input
  _delay_ms(500);
  uint8_t t;

  // useful for reconnection
  // messageString = "Fail"
  // while(strcmp(messageString, "Success") =! 0){
  // Transmit the string "ESP:restart\n" over UART
  const char *espRestartString = "ESP:restart\n";
  transmitStringOverUart(espRestartString);
  _delay_ms(500);

  // Transmit the string "ESP:connect\n" over UART
  const char *espConnectString = "ESP:connect\n";
  transmitStringOverUart(espConnectString);

  // Create a buffer to store the incoming message
  char message[BUFFER_SIZE];
  // Read the message from the UART interface
  char *messageString = readMessageFromUart(message, BUFFER_SIZE);
  _delay_ms(1000);
  // Check if the message is "Success" or "Fail"
    if (strstr(messageString, "\"Success\"") != NULL) {
      // Write "Connection successful" to the LCD screen
      writeMessageToLcd("1.Success");
    } else if (strstr(messageString, "\"Fail\"") != NULL) {
      //  Write "Connection failed" to the LCD screen
      writeMessageToLcd("1.Fail");
    } else {
      writeMessageToLcd(messageString);
    }
  // else return 0; //another message could be useful here

  // delay to make LCD message visible
  _delay_ms(2000);

  // Transmit the string "ESP:url:"http://192.168.1.250:5000/data"\n" over UART
  const char *espUrlString = "ESP:url:\"http://192.168.1.250:5000/data\"\n";
  transmitStringOverUart(espUrlString);

  // Read the message from the UART interface
  messageString = readMessageFromUart(message, BUFFER_SIZE);

  _delay_ms(1000);
  // Check if the message is "Success" or "Fail"
    if (strstr(messageString, "\"Success\"") != NULL) {
      // Write "Connection successful" to the LCD screen
      writeMessageToLcd("2.Success");
    } else if (strstr(messageString, "\"Fail\"") != NULL) {
      //  Write "Connection failed" to the LCD screen
      writeMessageToLcd("2.Fail");
    } else {
      writeMessageToLcd(messageString);
    }
  //  else return 0; //another message could be useful here
  int status_flag = 0;  //0 = OK, 1 = NURSECALL
  // delay to make LCD message visible
  _delay_ms(2000);
  while(1){
    //(a): read temp from DS18B20
    lcd_init();
    _delay_ms(50);

    //(c)
    /*  while (first == 0 || first > '9' || first < '0')
        ; // wait for 1st number to be pressed*/
    const char *status;
    if(status_flag == 0)
        status = "OK";
    else
        status = "NURSECALL";
    DDRD = 0xFF; // re init portd
    _delay_ms(15);
    _delay_ms(15);
    uint16_t temp;
    uint16_t temp_dec;
    int pressure;
    int pressure_dec;
    while(1) {
        _delay_ms(50);
        get_temp();   
        temp = (high & 0b0111);
        temp = (temp << 8) + low;
        temp_dec = temp & 0b1111;
        temp_dec = temp_dec*0.0625*1000;
        temp_dec = temp_dec/100;
        temp = temp * 0.0625;

        temp += 15; // adjust accordingly to temp on lab

        //(b) read value simulating pressure from POT0
        ADCSRA |= 0b01000000;
        while (1) { // wait for ADSC to become 0
          unsigned int t = ADCSRA & 0b01000000;
          if (t == 0)
            break;
        }
        pressure = ADC;
        pressure_dec = (pressure/5.1);
        pressure_dec = pressure_dec%10;
        pressure /= 51; // Convert to 0-20 cm scale
        int i=0;
        for(i=0; i<=100; i++){
            t = keypad_to_ascii();
            if (t != 0){break;}
        }
        if (t != 0) { // if a button was pressed update status
          if (t == '6' && status_flag == 0) {
            status = "NURSECALL";
            status_flag = 1;
            break;
          }
            else if(t == '#' && status_flag == 1){
                status = "OK";
                status_flag = 0;
                break;
            }            
        }
        
        if(status_flag == 0){            
                   if (pressure < 4 || pressure > 12){
                     status = "CHECKPRESSURE";
                   }
                   else if (temp < 34 || temp > 37) {
                     status = "CHECKTEMP";
                   }
                   else {
                     status = "OK";
                     status_flag = 0;
                   }
                 }
    DDRD = 0xFF;
    lcd_init();
    char payload[1024];
    snprintf(payload, sizeof(payload),
             "[{\"name\":\"temperature\",\"value\":\"%hu.%d\"},{\"name\":\"pressure\",\"value\":\"%d.%d\"},{\"name\":\"team\",\"value\":\"56\"},{\"name\":\"status\",\"value\":\"%s\"}]",temp,temp_dec,pressure, pressure_dec,status);
    
    char payload_comm[512];
    // prepare payload command string
    strcpy(payload_comm, "ESP:payload:");
    strcat(payload_comm, payload);
    strcat(payload_comm, "\n");
    // send payload command
    transmitStringOverUart(payload_comm);

    /*
    const char *payload_test = "ESP:payload:[{\"name\":\"team\",\"value\":\"56\"}]\n";
    transmitStringOverUart(payload_test);
    DDRD = 0xFF; // re init portd
     * */
    // Read the message from the UART interface
    messageString = readMessageFromUart(message, BUFFER_SIZE);

    _delay_ms(2000);
    // Check if the message is "Success" or "Fail"
      if (strstr(messageString, "\"Success\"") != NULL) {
        // Write "Connection successful" to the LCD screen
        writeMessageToLcd("3.Success");
      } else if (strstr(messageString, "\"Fail\"") != NULL) {
        //  Write "Connection failed" to the LCD screen
        writeMessageToLcd("3.Fail");
      } else {
        writeMessageToLcd(messageString);
      }

    _delay_ms(2000);
    // print values in LCD display
    //  Convert the first integer value to a string
    char value1String[10];
    sprintf(value1String, "%hu", temp);

    // Convert the second integer value to a string
    char value2String[10];
    sprintf(value2String, "%d", pressure);

    // Concatenate the two value strings together
    char message_1[20];
    strcpy(message_1, value1String);
    strcat(message_1, " ");
    strcat(message_1, value2String);

    // Write the concatenated message to the first line of the LCD screen
    writeMessageToLcd(message_1);
    // Set the cursor to the first character of the second line
    lcd_command(0xC0);
    _delay_ms(500);
    for (int i = 0; i < strlen(status); i++) {
      lcd_data(status[i]);
    }
    _delay_ms(1000);

    const char *espConnectString_1 = "ESP:transmit\n";
    transmitStringOverUart(espConnectString_1);

    // Read the message from the UART interface
    const char *messageStr = readMessageFromUart(message, BUFFER_SIZE);

    char server_msg[20];
    // prepare server response for LCD monitor
    strcpy(server_msg, "4.");
    strcat(server_msg, messageStr);
    // Print server response to LCD monitor
    lcd_init();
    _delay_ms(1000);
    writeMessageToLcd(server_msg);
    // insert delay to make message visible
    _delay_ms(2000);
  }
}
  return 0;
}