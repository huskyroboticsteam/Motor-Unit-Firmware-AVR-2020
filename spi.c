#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "spi.h"

/*Initalize SPI on PORTB
Parameters:
uint8_t master: if true, initalizes in master mode. Otherwise initalizes in slave mode
*/
void SPI_init(uint8_t master){
	/*Set the GPIO input/outputs*/
	if(master){
		DDRB = (1<<PB2) | (1<<PB1);
	} else {
		DDRB = (1<<PB3);
	}
	//Enable SPI. SCK = FCPU/128
	SPCR = (1<<SPE) | (1<<SPR1) | (1<<SPR0);
	if(master){
		SPCR |= (1<<MSTR); //Enable master mode
	}
}

/*Writes one byte to the SPI bus
Parameters:
uint8_t b: the byte to write
*/
void SPI_write(uint8_t b){
	SPDR = b;
	while(!(SPSR & (1<<SPIF))); //Wait for write to complete
}
/*Reads one byte from the SPI bus and returns it*/
uint8_t SPI_read(){
	while(!(SPSR & (1<<SPIF))); //Wait for receive to complete
	return SPDR;
}
/*Simultaniously reads and writes one byte on the SPI bus.
The received byte is returned
Parameters:
uint8_t b: the byte to transmit
*/
uint8_t SPI_rw(uint8_t b){
	SPDR = b;
	while(!(SPSR & (1<<SPIF))); //Wait for transaction to complete
	return SPDR;
}