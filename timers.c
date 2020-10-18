#include "config.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "timers.h"
#include "servo.h"
#include "util.h"

/*REV 2 note: the timer overflow frequency was increased from every 20mS to every 40mS*/

volatile uint32_t TOF_Cnt; //Timer1 overflow counter
volatile uint8_t PID_due;

#ifdef REV_2
ISR(TIMER1_OVF_vect){ //This should fire every 20mS
	TOF_Cnt++;
	if(TOF_Cnt % 8 == 0){
		update_LEDS((uint16_t)TOF_Cnt);
	}
	if(TOF_Cnt % 2 == 0){
		PID_due = 1;
	}
}
#else
ISR(TIMER1_OVF_vect){ //This should fire every 40mS
	TOF_Cnt++;
	if(TOF_Cnt % 4 == 0){
		update_LEDS((uint16_t)TOF_Cnt);
	}
	PID_due = 1;
	servo_high();
}
#endif

/*Set up the AVR's timers for PWM and time information*/
void setup_timers(){
	//Timer 0: Phase correct PWM, CLK/64
	TCCR0A = (1<<CS01) | (1<<CS00) | (1<<WGM00);
	TIMSK0 = (1 << TOIE0); // For velocity calculation. Fires every 2.05mS
	
	#ifdef REV_2
	//Timer 1: CTC, top=ICR1, TOP=OCR1A, CLK/64
	//This makes timer 1 increment every 8 microseconds at 8 Mhz or every 4uS at 16MHz
	TCCR1B = (1<<CS11) | (1<<CS10)  | (1<<WGM13) | (1<<WGM12);
	//ICR1 = 5000; //Count to 5000 before resetting
	TCCR1A = (1<<WGM11) | (1<<WGM10);
	OCR1A = 5000; //Count to 5000 before resetting
	#else
	//Timer 1: Fast PWM, TOP=OCR1A, CLK/64
	//This makes timer 1 increment every 8 microseconds at 8 Mhz or every 4uS at 16MHz
	TCCR1B = (1<<CS11) | (1<<CS10)  | (1<<WGM13) | (1<<WGM12);
	TCCR1A = (1<<WGM11) | (1<<WGM10);
	OCR1A = 10000; //Count to 10000 before resetting
	#endif
	TCNT1 = 0;
	TIMSK1 = (1 << TOIE1); //Enable interrupt on match
	
	
	//Timer 3: 10-bit phase correct PWM, CLK/64
	TCCR3A = (1<<WGM31) | (1<<WGM30);
	//TCCR3B = /*(1<<CS30) | */(1<<CS31);
	TCCR3B = (1<<CS30);

	TOF_Cnt = 0; //Clear the overflow counter
}

/*Returns the number of milliseconds since timer initialization*/
uint32_t get_mS(){
	uint16_t timer_ticks = TCNT1;
	#ifdef REV_2
	return (TOF_Cnt * 20) + (timer_ticks/250L);
	#else
	return (TOF_Cnt * 40) + (timer_ticks/250L);
	#endif
}

/*Returns the number of microseconds since system initialization*/
uint32_t get_uS(){
	uint16_t timer_ticks = TCNT1;
	#ifdef REV_2
	return (TOF_Cnt * 20000) + (timer_ticks << 2);
	#else
	return (TOF_Cnt * 40000) + (timer_ticks << 2);
	#endif
}

/*delay for the specified number of milliseconds*/
void delay_mS(uint16_t mS){
	uint32_t start = get_mS();
	while(get_mS() - start < mS);
}