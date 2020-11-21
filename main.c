#include "CANLibrary.h"
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <string.h>
#include <stdlib.h>

#include "pwm.h"
#include "encoder.h"
#include "motor.h"
#include "timers.h"
#include "adc.h"
#include "can.h"
#include "messaging.h"
#include "can.h"
#include "util.h"
#include "servo.h"
#include "usart.h"
#include "bss.h"

uint8_t get_dip_switch(){
	return (~PINA) & 0xF;
}

int main(){
	CANPacket m;
	DDRA = 0xF0;
	PORTA = 0xF0;
	setup_timers();
	_delay_ms(100);
	PORTA = 0;
	set_LED(0, 3);
	sei();
	usart_init(19200); //Debug serial
	set_LED(1, 3);
	delay_mS(666); //Delay so one can connect to debug serial
	uint16_t my_address = getLocalDeviceSerial();
	tprintf("adr=%X\n", my_address);
	delay_mS(250);
	InitCAN(DEVICE_GROUP_MOTOR_CONTROL, my_address);
	set_LED(2, 3);
	init_encoder();
	init_ADC();
	do_board_specific_setup(my_address);
	wdt_enable(WDTO_2S);
	init_motor();
	//enable_motor();
	delay_mS(500);
	set_LED(0, 0);
	set_LED(1, 0);
	set_LED(3, 0);
	while(1){
		if(PollAndReceiveCANPacket(&m) == 0){
			set_LED(3, 3);
			update_LEDS(get_mS()/40);
			handle_CAN_message(&m);
			set_LED(3, 0);
		}
		if(get_motor_mode() & MOTOR_MODE_ENABLED){
			set_LED(2, 3);
			set_LED(1, 0);
		} else {
			set_LED(1, 3);
			set_LED(2, 0);
		}
		motor_control_tick();
		wdt_reset();
	}
}