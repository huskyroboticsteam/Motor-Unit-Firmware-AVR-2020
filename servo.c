#include <avr/io.h>
#include <avr/interrupt.h>
#include <timers.h>
#include <config.h>
#include <servo.h>

#ifdef REV_2
void init_servo(){
	DDRB |= 1<<6;
	TCCR1A |= (1 << COM1B1);
//	TCCR1A &= ~(1 << COM1B0);
	OCR1B = 375;
}

void set_servo_position(int degrees){
	if(degrees > 179) degrees = 179;
	if(degrees < 1) degrees = 1;
	OCR1B = 250 + degrees + degrees / 3;
}

void servo_high(){}

#else
ISR(TIMER1_COMPB_vect){
	PORTC &= ~(1<<4);
}

void inline servo_high(){
	PORTC |= 1<<4;
}

void init_servo(){
	 TIMSK1 |= (1 << OCIE1B);
	 DDRC |= 1<<4;
	 OCR1B = 375;
}

void set_servo_position(int degrees){
	if(degrees > 180) return;
	if(degrees < 2) degrees = 2;
	OCR1B = degrees * 2 + degrees / 2;
}
#endif