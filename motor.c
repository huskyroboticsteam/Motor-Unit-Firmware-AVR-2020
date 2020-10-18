#include <avr/io.h>
#include "config.h"
#include "timers.h"
#include "pwm.h"
#include "adc.h"
#include "encoder.h"
#include "motor.h"
#include "can.h"
#include "messaging.h"
#include "usart.h"
#include "util.h"

uint16_t current;

int32_t motor_target_pos; //Motor target position/
int16_t motor_target_vel; //Motor target velocity
int32_t motor_max_pos; //The maximum position of the motor
int32_t last_pos_err; //Last position error (for dp)
int32_t pos_i; //Position integral
uint16_t Kp, Ki, Kd;
#ifdef DUAL_PID
int16_t last_vel_err; //Last velocity error (for dv)
int32_t vel_i; //Velocity integral
uint16_t Kp2, Ki2, Kd2;
#else
int32_t pid_target; //The current, actual target for the position PID
uint8_t tgt_inc; //The increment for the PID target
uint8_t slow; //"Slow" mode (less than 5 encoder ticks per 20mS)
#endif
uint16_t pid_runs; //The number of times the PID has run
int16_t motor_power;
uint8_t reverse = 0;
uint32_t last_set;

uint16_t motor_max_current;

uint8_t motor_mode; //Tracks the motor mode

/*Initalizes the motor controller and sets some reasonable defaults*/
void init_motor(){
	/*Turn off the PWM pin and set it to output*/
	MOTOR_PWM_PORT &= ~(1<<MOTOR_PWM);
	MOTOR_PWM_DDR |= (1<<MOTOR_PWM);
	/*Set direction pin to output and set motor direction to forward*/
	MOTOR_DIR_DDR |= (1<<MOTOR_DIR);
	MOTOR_DIR_PORT |= (1<<MOTOR_DIR);
	/*Set current sense pin to input*/
	MOTOR_CS_DDR &= ~(1<<MOTOR_CS);
	PORTE |= (1<<PE4);
	DDRE |= (1<<PE5);
	PORTD = 3; //Turn on limit switch pullup
	motor_max_pos = 1024; //BS this since we don't know yet
	motor_max_current = DEFAULT_MOTOR_CURRENT_LIMIT; 
	motor_target_pos = 0; //The encoder should start at 0 so this is a reasonable default
	pid_runs = 0;
	motor_power = 0;
	/*Set up the Kp, Ki and Kd terms to some reasonable defaults*/
	Kp = DEFAULT_Kp_1;
	Ki = DEFAULT_Ki_1;
	Kd = DEFAULT_Kd_1;
	#ifdef DUAL_PID
	Kp2 = DEFAULT_Kp_2;
	Ki2 = DEFAULT_Ki_2;
	Kd2 = DEFAULT_Kd_2;
	#else
	pid_target = 0;
	#endif
	set_target_velocity(DEFAULT_MOTOR_VELOCITY);
	init_encoder();
}

/*Sets the motor power directly without limit switch checking or updating variables
  Parameters:
  int16_t power: the motor power to set -1023 to +1023
  Negative values reverse the motor
*/
void set_motor_power_raw(int16_t power){
	if(power > 1023) power = 1023; /*Contrain power from -1023 to +1023*/
	if(power < -1023) power = -1023;
	#ifdef L298_MC /*L298 based motor controller*/
	if(power > 0){
		write_PWM(PE3, 0); /*Hardcoded pins; don't care while debugging*/
		write_PWM(PE4, power);
	} else {
		write_PWM(PE4, 0);
		write_PWM(PE3, -power);
	}
	#else /*Pololu motor controller*/
	if(power == 0 || !(motor_mode & MOTOR_MODE_ENABLED)){ //Shut down the motor if it isn't enabled or power is 0
		write_PWM(MOTOR_PWM, 0);
		MOTOR_PWM_PORT &= ~(1<<MOTOR_PWM);
		return;
	}
	if(power < 0){
		if(MOTOR_DIR_PORT & (1<<MOTOR_DIR)){
			MOTOR_DIR_PORT &= ~(1<<MOTOR_DIR); //Reverse
		}
		power = -power; //PWM is always positive
	} else {
		if(!(MOTOR_DIR_PORT & (1<<MOTOR_DIR))){
			MOTOR_DIR_PORT |= (1<<MOTOR_DIR); //Forward
		}
	}
	write_PWM(MOTOR_PWM, power);
	#endif
}

/*Sets the motor power more safely
  Parameters:
  int16_t power: the motor power to set -1023 to +1023
  Negative values reverse the motor*/
void set_motor_power(int16_t power){
	last_set = get_mS();
	if(!(motor_mode & MOTOR_MODE_ENABLED)){
		set_motor_power_raw(0);
		return;
	}
	if(reverse) power = -power;
	motor_power = power;
	uint8_t limit_sw = get_motor_limit_switch_state();
	if(limit_sw & 1){
		if(motor_power < 0){ 
			motor_power = 0;
		}
	}
	if(limit_sw & 2){
		if(motor_power > 0){
			motor_power = 0;
		}
	}
	//tprintf("%X\n", limit_sw);
	//tprintf("Setting motor power to %d\n", motor_power);
	set_motor_power_raw(motor_power);
}


/*Sets Kp for both PID loops. Value unchanged if a parameter is 0
Parameters:
uint16_t p1: The new value for Kp for position
uint16_t p2: The new value for Kp for velocity
*/
void set_Kp(uint16_t p1, uint16_t p2){
	if(p1 != 0){
		Kp = p1;
	}
	#ifdef DUAL_PID
	if(p2 != 0){
		Kp2 = p2;
	}
	#endif
}

/*Sets Ki for both PID loops. Value unchanged if a parameter is 0
Parameters:
uint16_t i1: The new value for Ki for position
uint16_t i2: The new value for Ki for velocity
*/
void set_Ki(uint16_t i1, uint16_t i2){
	if(i1 != 0){
		Ki = i1;
	}
	#ifdef DUAL_PID
	if(i2 != 0){
		Ki2 = i2;
	}
	#endif
}

/*Sets Kd for both PID loops. Value unchanged if a parameter is 0
Parameters:
uint16_t d1: The new value for Kd for position
uint16_t d2: The new value for Kd for velocity
*/
void set_Kd(uint16_t d1, uint16_t d2){
	if(d1 != 0){
		Kd = d1;
	}
	#ifdef DUAL_PID
	if(d2 != 0){
		Kd2 = d2;
	}
	#endif
}

int16_t avg;

/*Returns the motor current in milliamps*/
int16_t get_motor_current(){
	internalAREF(); //Use the 2.56V internal VRef for more precision
	#ifdef REV_2
	int16_t val = read_ADC(0) - 511;
	val = val * 34;
	avg = avg / 2 + val / 2;
	#else
	int16_t val = read_ADC(MOTOR_CS);
	//2.5 mV/unit. 8 Units/Amp
	if(val < 20){
		val = 0;
	} else {
		val -= 20; //Remove 50mV offset;
	//	val <<= 7; //Multiply by 128
		val <<= 6;
	}
	#endif
	return avg;
}

/*Returns true if the motor is stalled*/
uint8_t check_motor_stall(){
	return FALSE;/*
	static uint32_t overcurrent_since;
	if(get_motor_current() > motor_max_current){
		if(get_mS() - overcurrent_since > 500){ //Has the motor been over the limit for at least 500mS?
			return TRUE;
		}
	} else {
		overcurrent_since = get_mS(); //Not overcurrent so reset the timer
	}
	return FALSE;*/
}

/*Sets the maximum motor current in mA.
Parameters:
uin16_t current: The max motor current in milliamps
*/
void set_motor_current_limit(uint16_t current){
	motor_max_current = current;
}

/*Sets a target position for the motor*/
void set_target_position(int32_t position){
	//if(position < 0 || position > motor_max_pos) return; //Disallow setting motor to position outside the encoder range
	tprintf("setting target p to %l\n", position);
	//if(position < 0) return;
	if(int_abs(motor_target_pos - position) > 100){
		pos_i = 0;
	}
	motor_target_pos = position;
}

/*Sets a target velocity for the motor*/
void set_target_velocity(uint16_t velocity){
	tprintf("setting target v to %l\n", velocity);
	motor_target_vel = velocity;
	#ifndef DUAL_PID
	if(velocity / 25 < 5){ //Less than 5 encoder ticks per PID tick
		slow = 1; //Slow mode
		tgt_inc = 125 / velocity + ((125 % velocity) > velocity/2); //Integer division + rounding
	} else {
		slow = 0;
		tgt_inc = velocity / 25; //Otherwise, set up the increment
	}
	#endif
	tprintf("slow = %d, tgt_inc = %d\n", slow, tgt_inc);
}

/*Gets the target position for the motor*/
int32_t get_target_position(){
	return motor_target_pos;
}

/*Gets the target velocity for the motor*/
int32_t get_target_velocity(){
	return motor_target_vel;
}

/*Gets the current motor velocity*/
int16_t get_motor_velocity(){
	return get_encoder_velocity();
}

void index_motor(){
	motor_mode |= MOTOR_MODE_INDEX;
	if(motor_target_vel == 0){
		set_target_velocity(DEFAULT_MOTOR_VELOCITY); //Indexing won't work if this is 0.
	}
	motor_target_pos = -262144; //Big negative number. We should hit the limit switch before this
}
#ifdef DEBUG
int16_t av;
#endif
//int last;
int32_t last = 0;
/*Executes one tick of the motor control system. Call this in a loop!*/
void motor_control_tick(){
	if(!(motor_mode & MOTOR_MODE_PID)){ //If the PID is disabled, simply unset the PID due flag
		PID_due = 0;
		int32_t t = get_encoder_ticks();
		if(t != last){
			tprintf("%l\n", t);
			last = t;
		}
		if(get_mS() - last_set > MOTOR_SET_TIMEOUT){
//			set_motor_power(0);
			motor_power = 0;
		}
	}
	if(check_motor_stall() || !(PINE & (1<<PE4))){ //Motor stall or fault pin asserted from motor driver
		motor_power = 0;
		disable_motor();
		//send_CAN_error(CAN_ERROR_OVERCURRENT, /*get_motor_current()>>10*/0);
		set_LED(0, 1);
	}
	if(motor_mode & MOTOR_MODE_PID && PID_due){ //Are we supposed to run the PID?
	//	tprintf("period=%d\n", get_mS() - last);
	//	last=get_mS();
		#ifdef DUAL_PID
		/*Velocity PID*/
		int16_t vel = get_motor_velocity();
		int16_t errorv; 
		if(get_encoder_ticks() < motor_target_pos){ //Check what direction we need to go. If the enc. ticks are less than the target, we need to go forward
			errorv = motor_target_vel - vel;
		} else { //Otherwise we're going backwards
			errorv = (-motor_target_vel) - vel; //We need the velocity to be negative if the motor needs to run backwards
		}
		int16_t dv = errorv - last_vel_err; //d term
		last_vel_err = errorv; 
		int32_t mvp = (errorv*Kp2)/20 + (vel_i*Ki2)/20 + (dv*Kd2)/20; //Compute the motor power for the velocity loop
		//int32_t mvp = errorv/2 + vel_i / 8 + dv/10;
	//	tprintf("%d %d %d %d %d %d | ", (int16_t)motor_target_vel, (int16_t)vel, (int16_t)errorv, (int16_t)dv, (int16_t)vel_i, (int16_t)mvp);
		if(errorv < -16 || errorv > 16)
			vel_i += errorv; //Update the integral term. Ignore small steady state errors
		if(vel_i > 16384) vel_i = 16384; //Constrain the intergral term to prevent integral wind-up
		if(vel_i < -16384) vel_i = -16384;
		if(mvp > 1023) mvp = 1023;
		if(mvp < -1023) mvp = -1023;
		uint32_t pid_target = motor_target_pos;
		#else /*Single PID*/
		if(slow){ //Slow mode. Change the target at most 5 ticks per PID loop
			if(pid_runs % tgt_inc == 0){ //Are we supposed to increment this tick?
				if(motor_target_pos > pid_target){ //The real target is bigger than the PID's current target
					pid_target += min(5, motor_target_pos - pid_target); //Increment at most 5 ticks but don't go over the real target
				} else if(motor_target_pos < pid_target){ //The real target is smaller than the PID's current target
					pid_target -= min(5, pid_target - motor_target_pos); //Deincrement at most 5 ticks but don't go under the real target
				}
			}
		} else { //Fast mode
			if(motor_target_pos > pid_target){ //The real target is bigger than the PID's current target
					pid_target += min(tgt_inc, motor_target_pos - pid_target); //Increment by at most tgt_inc
			} else if(motor_target_pos < pid_target){ //The real target is smaller than the PID's current target
					pid_target -= min(tgt_inc, pid_target - motor_target_pos); //Deincrement by at most tgt_inc
			}
		}
		#endif
		/*Position PID*/
		int32_t pos = get_encoder_ticks();
		#ifndef DUAL_PID
		if(pos < pid_target - 128 || pos > pid_target + 128){ /*If there is a large error, the PID is out of lock*/
		//	tprintf("Assinging PID target %l, %l\n", pid_target, pos);
	//		pid_target = pid_target/2 + pos/2; //Adjust the PID target closer to the actual motor position to keep it from racing to it
		}
		#endif
		int32_t errorp = pos - pid_target; //P
		int32_t dp = errorp - last_pos_err; //D
		last_pos_err = errorp;
		if(int_abs(errorp) > 4) //Ignore small steady state errors
			pos_i += errorp; //I
		if(int_abs(pos - motor_target_pos) < 4){
			motor_power = 0;
		} else {
			if(pos_i > 768) pos_i = 768; /*Constrain integral to avoid integral wind-up*/
			if(pos_i < -768) pos_i = -768;
			int32_t mpp = (errorp*Kp)/20 + (pos_i*Ki)/20 + (dp*Kd)/20;
			if(mpp > 384) mpp = 384; /*Clamp the motor power to the accepted range of -1023 to +1023*/
			if(mpp < -384) mpp = -384;		
			motor_power = mpp; //Set the motor power to the output
			#ifdef DUAL_PID
			if(int_abs(errorp) > 100){ //If there is a large error, let the velocity PID take over
				motor_power = mvp;
			}
			#endif
		}
		#ifdef DEBUG
		av = (av*9)/10 + get_motor_velocity()/10;
		//tprintf("%l %l %l %d %d %d %d %d\n", (int32_t)motor_target_pos, (int32_t)pid_target, (int32_t)pos, (int16_t)errorp, (int16_t)dp, (int16_t)pos_i, (int16_t)motor_power, av);
		//tprintf("%d %d %d %d\n", (int16_t)pid_target, (int16_t)pos, (int16_t)av, (int16_t)motor_power);
		tprintf("%d %d\n", (int16_t)pos, motor_power);
		//last_mS = get_mS();
		#endif
		pid_runs++;
		PID_due = 0; //We're done running the PID for now
	}
	uint8_t limit_sw = get_motor_limit_switch_state();
	if(limit_sw & 1){
//		reset_encoder();
		if(motor_mode & MOTOR_MODE_INDEX) /*We're in index mode and hit the loewr limit*/
			motor_mode &= ~MOTOR_MODE_INDEX; //we hit the limit switch. leave indexing mode
			//motor_target_pos = 262144; //Now try to find the upper limit
		if(motor_target_pos < 0)
			motor_target_pos = 0;
		if(motor_power < 0){ 
			motor_power = 0;
			set_motor_power_raw(0);
		}
	}
	if(limit_sw & 2){
		reset_encoder();
		//motor_max_pos = get_encoder_ticks();
	/*	if(motor_target_pos > motor_max_pos){
			motor_target_pos = motor_max_pos;
		}*/
		if(motor_mode & MOTOR_MODE_INDEX){ //Are we in index mode?
			motor_mode &= ~MOTOR_MODE_INDEX; //Leave indexing mode
		}
		if(motor_power > 0){
			motor_power = 0;
			set_motor_power_raw(0);
		}
	}
	set_motor_power_raw(-motor_power);		
}

/*Enables the motor*/
void enable_motor(){
	motor_mode |= MOTOR_MODE_ENABLED;
	set_LED(0, 0);
}

/*Disables the motor*/
void disable_motor(){
	set_motor_power_raw(0);
	motor_mode &= ~MOTOR_MODE_ENABLED;
}

/*Sets the motor mode*/
void set_motor_mode(uint8_t mode){
	motor_mode = mode;
}

/*Gets the motor mode*/
uint8_t get_motor_mode(){
	return motor_mode;
}

/*Gets the state of the limit switches*/
uint8_t get_motor_limit_switch_state(){
	//return ((~PIND) & 0x03); //Get GPIO data
	#ifdef REV_2
	return ((~PIND & 6) >> 1);
	#endif
	return (~PIND & 2) | ((~PINC & 2) >> 1);
}

/*Gets the motor maximum position*/
uint32_t get_motor_max_position(){
	return motor_max_pos;
}

void set_motor_reverse(uint8_t r){
	reverse = r;
}