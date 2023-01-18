#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/delay.h>

#define BUFFER_SIZE 256

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
  _delay_ms(10);
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

  lcd_init(); // init LCD
  _delay_us(500);

  // Initialize the UART interface with a baud rate of 9600
  usart_init(103);
  _delay_us(50);

const char *messageString;
// Create a buffer to store the incoming message
char message[BUFFER_SIZE];
// Transmit the string "ESP:restart\n" over UART
const char *espRestartString = "ESP:restart\n";
transmitStringOverUart(espRestartString);
_delay_ms(500);
// Transmit the string "ESP:connect\n" over UART
const char *espConnectString = "ESP:connect\n";
transmitStringOverUart(espConnectString);
// Read the message from the UART interface
messageString = readMessageFromUart(message, BUFFER_SIZE);

if (strstr(messageString, "\"Success\"") != NULL) {
  // Write "Connection successful" to the LCD screen
  writeMessageToLcd("1.Success");
} else if (strstr(messageString, "\"Fail\"") != NULL) {
  //  Write "Connection failed" to the LCD screen
  writeMessageToLcd("1.Fail");
} else {
  writeMessageToLcd(messageString);
}

// delay to make LCD message visible
_delay_ms(6000);

// reset buffer
memset(message, 0, sizeof(message));

// Transmit the string "ESP:url:"http://192.168.1.250:5000/data"\n" over
// UART
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
  // if (strcmp(messageString, "Fail") == 0)
  //  Write "Connection failed" to the LCD screen
  writeMessageToLcd("2.Fail");
} else
  writeMessageToLcd(messageString); // another message could be useful here
}