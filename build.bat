set MCU=at90can32

del avr_mc.elf avr_mc.hex
avr-gcc -c timers.c -o timers.o -Iinclude -I. -mmcu=%MCU% -O3
avr-gcc -c pwm.c -o pwm.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c adc.c -o adc.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c can.c -o can.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c encoder.c -o encoder.o -Iinclude -I. -mmcu=%MCU% -O3
avr-gcc -c motor.c -o motor.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c usart.c -o usart.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c spi.c -o spi.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c messaging.c -o messaging.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c util.c -o util.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c servo.c -o servo.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc -c main.c -o main.o -Iinclude -I. -mmcu=%MCU% -Os
avr-gcc main.o messaging.o spi.o usart.o motor.o encoder.o can.o adc.o pwm.o timers.o servo.o util.o -mmcu=%MCU% -o avr_mc.elf
avr-size -C avr_mc.elf --mcu=%MCU%