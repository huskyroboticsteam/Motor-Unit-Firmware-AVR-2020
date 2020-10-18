#ifndef TIMERS_H
#define TIMERS_H

void setup_timers();
uint32_t get_mS();
uint32_t get_uS();
void delay_mS(uint16_t mS);

extern volatile uint8_t PID_due;

#endif