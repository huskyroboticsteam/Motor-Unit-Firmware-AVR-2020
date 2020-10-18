#include <avr/io.h>
#include "config.h"
#include "usart.h"
#include "util.h"

uint8_t LED_states;
uint16_t ticks_per_degree_x10 = 100;

int32_t min(int32_t a, int32_t b){
	return (a < b)? a:b;
}

int32_t max(int32_t a, int32_t b){
	return (a > b)? a:b;
}

/*Sets the mode of the debug LEDs
Parameters:
uint8_t L: the number of the LED to set (0-3)
uint8_t state: the mode for the LED:
	0 -> Off
	1 -> Slow flashing
	2 -> Quick flashing
	3 -> Steady on
*/
void set_LED(uint8_t L, uint8_t state){
	if(L > 3 || state > 3) return;
	if(state == 1 || state == 2){
		state = 3 - state;
	}
	L <<= 1;
	LED_states = (LED_states & ~(3 << L)) | (state << L);
	//tprintf("LED:%d\n", LED_states);
}

/*Integer implementation of abs()*/
uint16_t int_abs(int16_t x){
	return x<0? -x: x;
}

/*Call this from a timer interrupt to update the debug LEDs
Parameters:
uint16_t count: The timer count
*/
void inline update_LEDS(uint16_t count){
	int i;
	uint8_t v;
	#ifdef REV_2
	count /= 2;
	#endif
	if(!LED_states){
		PORTA = 0;
		return;
	}
	for(i = 0;i < 4;i++){
		v = (LED_states >> (2*i)) & 3;
		if(v == 0 || ((count % (1+(v*2))) && v != 3)){
			PORTA &= ~(1 << (i+4));
		} else {
			PORTA |= 1 << (i+4);
		}
	}
}

/*Reads a string from uart
Parameters:
char *buffer: the buffer to put the string
char until: The deliminating character
*/
void read_string(char *buffer, char until){
	char c = -1;
	int i = 0;
	while(c != until){
		c = usart_read_char();
		tprintf("%c", c);
		buffer[i++] = c;
	}
	buffer[i-1] = 0;
}

/*Converts tenths of degrees (degrees * 10) to encoder ticks
Parameters:
int16_t d: the number of degrees * 10
Returns:
the number of encoder ticks
*/
int16_t tendeg_to_ticks(int16_t d){
	return ((int32_t)d * (int32_t)ticks_per_degree_x10) / 100L;
}

/*Converts degrees to encoder ticks
Parameters:
int16_t d: the number of degrees
Returns:
the number of encoder ticks
*/
int16_t deg_to_ticks(int16_t d){
	return ((int32_t)d * (int32_t)ticks_per_degree_x10) / 10L;
}

/*Sets a new ratio of ticks to degrees.
Parameters:
uint16_t tpd: the number of encoder ticks in 10 degrees of rotation
*/
void set_ticks_per_10degrees(uint16_t tpd){
	ticks_per_degree_x10 = tpd;
}