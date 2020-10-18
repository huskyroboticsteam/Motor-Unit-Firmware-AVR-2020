#ifndef ADC_H
#define ADC_H

void init_ADC();
void internalAREF();
void externalAREF();
uint16_t read_ADC(uint8_t pin);
uint16_t get_voltage();

#endif