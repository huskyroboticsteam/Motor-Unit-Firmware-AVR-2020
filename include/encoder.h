#ifndef ENCODER_H
#define ENCODER_H

int32_t get_encoder_ticks();
int16_t get_encoder_velocity();
void set_encoder_ticks(int32_t ticks);
void init_encoder();
void reset_encoder();

#endif