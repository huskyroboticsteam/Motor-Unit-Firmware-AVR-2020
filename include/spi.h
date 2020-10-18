#ifndef SPI_H
#define SPI_H

void SPI_init(uint8_t master);
void SPI_write(uint8_t b);
uint8_t SPI_read();
uint8_t SPI_rw(uint8_t b);

#endif