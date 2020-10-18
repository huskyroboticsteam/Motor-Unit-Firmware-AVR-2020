#include <avr/io.h>
#include <avr/interrupt.h>
#include "encoder.h"
#include "timers.h"

//Define "DEBOUNCE" to use the software debouncer
//#define DEBOUNCE

volatile int32_t encoder_ticks; //Raw encoder ticks
volatile int16_t vticks; //Ticks for velocity computation
volatile int16_t raw_velocity; //Raw velocity
int16_t old_velocity; //The last velocity reading
volatile uint8_t gate_control, gate_control_top; //For dynamically adjusted gate time for velocity computation
//volatile uint8_t run_PID;

#ifdef DEBOUNCE
volatile uint16_t lastA, lastB;
#endif

ISR(TIMER0_OVF_vect){
	gate_control++;
	if(gate_control == gate_control_top){
		raw_velocity = vticks;
		vticks = gate_control = 0;
	}
}

/*Returns the total number of encoder ticks since the last reset*/
int32_t get_encoder_ticks(){
	//return -19;
	//return 0;
	return encoder_ticks;
	//return 255;
}

/*Sets the encoder counter to the specified value*/
void set_encoder_ticks(int32_t ticks){
	encoder_ticks = ticks;
}

/*Initializes the encoder with A on PE6 and B on PE7*/
void init_encoder(){
	EIMSK &= 0x3F; //Disable pin change interrupts
	DDRE &= 0x3F; //Set PE6, PE7 as inputs
	PORTE |= 0xC0; //Enable pullups on PE6, PE7
	encoder_ticks = 0; //Reset count
	gate_control_top = 24; //Short gate time
	EICRB |= 0x50; //Enable pin change interrupt on PE6, PE7
	EICRA = 0;
	EIMSK |= 0xC0;
}

/*Resets the encoder count*/
void reset_encoder(){
	EIMSK &= 0x3F; //Disable pin change interrupts
	encoder_ticks = 0; //Reset count
	vticks = 0;
	raw_velocity = 0;
	gate_control = 0;
	EIMSK |= 0xC0; //Enable pin change interrupt on PE6, PE7
}

/*Returns the encoder velocity in ticks/second*/
int16_t get_encoder_velocity(){
	int16_t rv = raw_velocity;
	int16_t velocity;
	if(gate_control_top == 24){
		if(rv < 5 && rv > -5){ //If it's slow, increase the gate time
			gate_control_top = 122;
			//tprintf("GCT=122\n");
		}
		velocity = (rv * 60)/4 + old_velocity/4; //Compute velocity and low pass filter
		old_velocity = velocity;
		return velocity;
	} else if(gate_control_top == 122){ //If it's fast, decrease gate time
		if(rv > 80 || rv < -80){
			gate_control_top = 24;
			//tprintf("GCT=24\n");
		}
		velocity = (rv * 12)/4 + old_velocity/4; //Compute velocity and LPF
		old_velocity = velocity;
		return velocity;
	}
	return -1;
}

ISR(INT6_vect){ //PE6, A
	#ifdef DEBOUNCE
	uint16_t tc = TCNT1;
	if(tc > lastA && tc - lastA < 400) return;
	lastA = tc;
	#endif
	uint8_t state = PINE;
	if(state & (1<<PE6)){ //A rising
		if(state & (1<<PE7)){ //B high
			encoder_ticks++;
			vticks++;
		} else { //B low
			encoder_ticks--;
			vticks--;
		}
	} else { //A falling
		if(state & (1<<PE7)){ //B high
			encoder_ticks--;
			vticks--;
		} else { //B low
			encoder_ticks++;
			vticks++;
		}
	}
}

ISR(INT7_vect){ //PE7, B
	#ifdef DEBOUNCE
	uint16_t tc = TCNT1;
	if(tc > lastB && tc - lastB < 400) return;
	lastB = tc;
	#endif
	uint8_t state = PINE;
	if(state & (1<<PE7)){ //B rising
		if(state & (1<<PE6)){ //A high
			encoder_ticks--;
			vticks--;
		} else { //A low
			encoder_ticks++;
			vticks++;
		}
	} else { //B falling
		if(state & (1<<PE6)){ //A high
			encoder_ticks++;
			vticks++;
		} else { //A low
			encoder_ticks--;
			vticks--;
		}
	}
}