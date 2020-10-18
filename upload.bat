avr-objcopy avr_mc.elf -O ihex avr_mc.hex
avrdude -p at90can32 -c avrisp -P COM10 -b 38400 -V -U avr_mc.hex
