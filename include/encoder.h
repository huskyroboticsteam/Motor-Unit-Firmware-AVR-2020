#ifndef ENCODER_H
#define ENCODER_H

extern volatile uint32_t ppjr;

int32_t get_encoder_ticks();
int16_t get_encoder_velocity();
int32_t ticks_to_angle(int32_t ticks);
int32_t angle_to_ticks(int32_t angle);
void set_encoder_ticks(int32_t ticks);
void init_encoder();
void reset_encoder();

#endif