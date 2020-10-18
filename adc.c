#include "config.h"
#include <avr/io.h>
#include "adc.h"
#include "timers.h"

/*Initialize the ADC and prepare it for reading*/
void init_ADC(){
	ADMUX = (1<<6); //5V VREF
	#if F_CPU < 10000000
	ADCSRA = (1<<ADPS2) | (1<<ADPS0); //Up to 10MHz, prescale by 32
	#else
	ADCSRA = (1<<ADPS2) | (1<<ADPS1); //For faster clocks, prescale by 64
	#endif
	ADCSRA |= (1<<ADEN); //Enable the ADC
}

/*Reads one sample from the ADC and returns it
  Paramters:
  uint8_t pin: The ADC pin number to read
  Return value:
  The 10-bit ADC result
*/
uint16_t read_ADC(uint8_t pin){
	uint8_t l,h;
	ADMUX = (ADMUX & 0xC0) | (pin & 7); //Setup ADC, preserve REFS0
	ADCSRA |= (1<<ADSC); //Start the conversion
	while(ADCSRA & (1<<ADSC)); //Wait for conversion
	l = ADCL;  //Read and return 10 bit result
	h = ADCH;
	return (h << 8)|l; 
}

/*Switches to the internal (2.56V) Vref*/
void internalAREF(){
	if((ADMUX & 0xC0) != 0xC0){
		ADMUX |= 0xC0;
		delay_mS(5);
		read_ADC(0); //As per the datasheet, discard the first reading after changing AREF
	}
}

/*Uses VIN for Vref (5 or 3.3V)*/
void externalAREF(){
	if((ADMUX & 0xC0) == 0xC0){
		ADMUX &= ~0xC0;
		ADMUX = (1<<6);
		delay_mS(5);
		read_ADC(0);
	}
}

/*Gets the voltage on the +24V line and returns the results in mV*/
uint16_t get_voltage(){
	internalAREF();
	uint16_t val = read_ADC(VS_PIN);
	//2.5mV/unit, 16:1 resistor ratio
	return (val << 5) + (val << 3); //fast multiply by 40
}