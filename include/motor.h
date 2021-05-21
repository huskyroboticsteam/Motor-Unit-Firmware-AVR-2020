#ifndef MOTOR_H
#define MOTOR_H

#include <config.h>

#define MOTOR_MODE_INDEX 4
#define MOTOR_MODE_PID 2
#define MOTOR_MODE_ENABLED 1

void init_motor();
//uint8_t PID_due();
void set_motor_power(int16_t power);
void set_motor_power_raw(int16_t power);
int16_t get_motor_current();
void set_motor_current_limit(uint16_t current);
uint8_t check_motor_stall();
void enable_motor();
void disable_motor();
uint8_t get_motor_mode();
void set_motor_mode(uint8_t mode);
void set_target_position(int32_t position);
int16_t get_motor_velocity();
int32_t get_target_position();
void set_Kp(uint16_t p);
void set_Ki(uint16_t i);
void set_Kd(uint16_t d);
void index_motor();
void motor_control_tick();
uint8_t get_motor_limit_switch_state();
uint32_t get_motor_max_position();
void set_motor_reverse(uint8_t r);

#endif