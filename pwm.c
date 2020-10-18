#include "config.h"
#include <avr/io.h>
#include "usart.h"
#include "pwm.h"

/*Writes a PWM value to the specified pin. Currently only Timer3/PORTE supported
  Parameters:
  uint8_t pin: The pin to set the PWM on (PE3, PE4, PE5)
  uint16_t pwm: 10-bit PWM value
*/
//void write_PWM(uint8_t port, uint8_t pin, uint16_t pwm){
void write_PWM(uint8_t pin, uint16_t pwm){
	pwm &= 1023;
	switch(pin){
		case PE3:
			TCCR3A |= (1 << COM3A1);
			OCR3A = pwm;
			break;
		case PE4:
			TCCR3A |= (1 << COM3B1);
			OCR3B = pwm;
			break;
		case PE5:
			TCCR3A |= (1 << COM3C1);
			OCR3C = pwm;
			break;
	}
}

/*Disables PWM on the specified pin
  Parameters:
  uint8_t pin: the pin to disable pwm on
*/
//void disable_PWM(uint8_t port, uint8_t pin){
void disable_PWM(uint8_t pin){
	switch(pin){
		case PE3:
			TCCR3A &= ~(1 << COM3A1);
			OCR3A = 0;
			break;
		case PE4:
			TCCR3A &= ~(1 << COM3B1);
			OCR3B = 0;
			break;
		case PE5:
			TCCR3A &= ~(1 << COM3C1);
			OCR3C = 0;
			break;
	}
}