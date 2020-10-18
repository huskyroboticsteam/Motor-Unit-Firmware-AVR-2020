#ifndef USART_H
#define USART_H

#include "config.h"

void usart_init(uint32_t baud);
void usart_end();
uint8_t usart_read(char *buf, uint8_t count);
char usart_read_char();
void usart_write(char *buf, uint8_t count);
void usart_write_char(char c);
void usart_write_string(char *s);
int usart_available();
void tprintf(const char *fmt, ...);

#endif