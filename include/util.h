#ifndef UTIL_H
#define UTIL_H

int32_t min(int32_t a, int32_t b);
int32_t max(int32_t a, int32_t b);
uint16_t int_abs(int16_t x);
void set_LED(uint8_t L, uint8_t state);
void update_LEDS(uint16_t count);
void read_string(char *buffer, char until);
int16_t tendeg_to_ticks(int16_t d);
void set_ticks_per_10degrees(uint16_t tpd);
int16_t deg_to_ticks(int16_t d);

#define false 0
#define true 1
#define FALSE 0
#define TRUE 1

#endif