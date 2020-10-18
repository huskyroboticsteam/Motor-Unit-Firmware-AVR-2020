#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include "usart.h"
#include "util.h"

volatile char usart_TX_buf[USART_TX_BUF_SZ];
volatile char usart_RX_buf[USART_RX_BUF_SZ];
volatile uint8_t txbp, txend, rxbp;

ISR(USART1_RX_vect){
	if(rxbp > USART_RX_BUF_SZ){
		rxbp = USART_RX_BUF_SZ;
		return;
	}
	usart_RX_buf[rxbp++] = UDR1;
}
ISR(USART1_UDRE_vect){
	UDR1 = usart_TX_buf[txbp++];
	if(txbp >= txend || txbp > USART_TX_BUF_SZ){
		UCSR1B &= ~(1 << UDRIE1);
		txbp = txend = 0;
	}
}

/*Reads bytes from the USART into the specified buffer
Parameters:
char *buf: The buffer to receive the data
uint8_t count: The number of bytes to read;
Returns:
Number of bytes recieved
*/
uint8_t usart_read(char *buf, uint8_t count){
	uint8_t icfg = UCSR1B;
	while(rxbp < 1);
	UCSR1B &= ~(1 << RXCIE1); //Disable USART RX interrupt during operation
	uint8_t c = min(rxbp, count);
	memcpy((void*)buf, (void*)usart_RX_buf, c);
	memmove((void*)usart_RX_buf, (void*)usart_RX_buf + c, USART_RX_BUF_SZ - c);
	rxbp -= c;
	UCSR1B = icfg;
	return c;
}

/*Writes data out to the USART
Parameters:
char *buf: The data to write
uint8_t count: How many bytes to write
*/
void usart_write(char *buf, uint8_t count){
	volatile uint8_t c;
	if(txend){UCSR1B |= (1 << UDRIE1);}
	while(count){
		while(txend >= USART_TX_BUF_SZ-1);
		c = min(USART_TX_BUF_SZ - txend, count);
		memcpy((void*)usart_TX_buf + txend, buf, c);
		txend += c;
		count -= c;
		UCSR1B |= (1 << UDRIE1);
	}
}

/*Initialize the USART1
Parameters:
uint16_t baud: The baud rate to use
*/
void usart_init(uint32_t baud){
	rxbp = 0;
    txbp = txend = 0;
	UCSR1A = 0;
	UBRR1 = ((F_CPU/16L)/baud)-1;
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10); //8-bit, 1 stop bit, no parity
	UCSR1B = (1<<RXEN1) | (1<<TXEN1) | (1<<RXCIE1); //Enable recieve, transmit and interrupts
}

/*Disables USART*/
void usart_end(){
	UCSR1B = 0;
}

/*Writes one character to the USART
Parameters:
char c: the character to write*/
void usart_write_char(char c){
	if(c == '\n'){
		usart_write_char('\r');
	}
	usart_write(&c, 1);
}

/*Writes a string to the USART
Parameters:
char *s: The string to write*/
void usart_write_string(char *s){
	usart_write(s, strlen(s));
}

/*Reads one character (byte) from the USART recieve buffer. If the buffer is empty, it returns 0*/
char usart_read_char(){
	char c;
	usart_read(&c, 1);
	return c;
}

/*Returns the number of bytes waiting in the USART receive buffer*/
int usart_available(){
	return rxbp;
}

/*Tiny implementation of prinf()*/
void tprintf(const char *fmt, ...){
	va_list va;
	va_start(va, fmt);
	int i = 0;
	int32_t l;
	char buf[9],c;
	for(i = 0;c=fmt[i];i++){
		if(c != '%'){
			usart_write_char(c);
		}
	if(c == '\\'){
		c = fmt[++i];
		switch(c){
			case '\\':
				usart_write_char('\\');
				break;
			case '\n':
				usart_write_char('\n');
				break;
		}
	}
	if(c == '%'){
		c=fmt[++i];
		switch(c){
			case 'l':
				l = va_arg(va, int32_t);
				if(l > 32767){
					itoa(l/10000, buf, 10);
					usart_write_string(buf);
					l %= 10000;
					if(l < 1000){
						usart_write_char('0');
					}
					if(l < 100){
						usart_write_char('0');
					}
					if(l < 10){
						usart_write_char('0');
					}
				}
				itoa(l, buf, 10);
				usart_write_string(buf);
				break;
			case 'X':
				itoa(va_arg(va, int32_t), buf, 16);
				usart_write_string(buf);
				break;
			case 'd':
				itoa(va_arg(va, int), buf, 10);
				usart_write_string(buf);
				break;
			case 's':
				usart_write_string(va_arg(va, char*));
				break;
			case 'c':
				usart_write_char(va_arg(va, int));
				break;
			default:
				usart_write_char('?');
				break;
			}
		}
	}
}