CC=avr-gcc
CFLAGS += -DF_CPU=16000000UL
CFLAGS += -mmcu=atmega644p
CFLAGS += -O3
MCU=atmega644p
all: main2.o
	avr-gcc -mmcu=atmega644p -o firmware $^
	avr-objcopy -O ihex -R .eeprom firmware firmware.hex
	avr-size firmware

clean:
	rm -f firmware firmware.hex main2.o

program:
	avrdude -c usbasp -p ${MCU} -P usb -U flash:w:firmware.hex:i


