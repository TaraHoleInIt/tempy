PROJECT=tempy

MCU=attiny402
FREQ=3333333

OBJCOPY=avr-objcopy
CC=avr-gcc
LD=avr-ld

CFLAGS=-g -Wall -Os -DF_CPU=$(FREQ) -D__AVR_ATtiny406__ -mmcu=$(MCU) -Wl,-g,-flto -Wa,-mgcc-isr
LDFLAGS=-g -flto

OBJS=build/main.o

default: all

build/%.o: src/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

build/%.o: src/%.S
	$(CC) -c -o $@ $< $(CFLAGS)

build/$(PROJECT).elf: $(OBJS)
	$(CC) -o build/$(PROJECT).elf $(CFLAGS) $(OBJS)

build/$(PROJECT).hex: build/$(PROJECT).elf
	$(OBJCOPY) -O ihex build/$(PROJECT).elf build/$(PROJECT).hex

size: build/$(PROJECT).elf
	avr-size build/$(PROJECT).elf
	avr-nm --size-sort --print-size --radix=d build/$(PROJECT).elf

flash:
	avrdude -c jtag2updi -P /dev/ttyS3 -F -p t402w -U flash:w:build/$(PROJECT).hex

all: clean build/$(PROJECT).hex size

.PHONY: clean

clean:
	rm -rfv build/*

