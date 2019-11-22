PROJECT=tiny0test

MCU=attiny402
FREQ=3333333

SIZE=avr-size
OBJCOPY=avr-objcopy
CC=avr-gcc
LD=avr-ld

CFLAGS=-g -Wall -Os -DF_CPU=$(FREQ) -D__AVR_ATtiny402__ -mmcu=$(MCU) -Wl,-g,-flto -Wa,-mgcc-isr
LDFLAGS=-g -flto

OBJS=build/main.o

default: all

build/%.o: src/%.c
	$(CC) -c -o $@ $< $(CFLAGS)

build/$(PROJECT).elf: $(OBJS)
	$(CC) -o build/$(PROJECT).elf $(CFLAGS) $(OBJS)

build/$(PROJECT).hex: build/$(PROJECT).elf
	$(OBJCOPY) -O ihex build/$(PROJECT).elf build/$(PROJECT).hex

size: build/$(PROJECT).elf
	$(SIZE) build/$(PROJECT).elf

all: clean build/$(PROJECT).hex size

.PHONY: clean

clean:
	rm -rfv build/*

