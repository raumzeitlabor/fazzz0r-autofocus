CC=avr-gcc
CFLAGS += -DF_CPU=12000000UL
CFLAGS += -mmcu=atmega644
CFLAGS += -Os
MCU=atmega644
all: main.o
	avr-gcc -mmcu=atmega644 -o firmware $^
	avr-objcopy -O ihex -R .eeprom firmware firmware.hex
	avr-size firmware

clean:
	rm firmware firmware.hex main.o

program:
	avrdude -c usbasp -p ${MCU} -P usb -U flash:w:firmware.hex:i


