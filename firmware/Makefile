# Makefile - firmware do controle de nivel (ATmega328P @ 16 MHz)
#
#   make            compila build/nivel.elf e build/nivel.hex
#   make size       uso de flash/RAM
#   make flash      grava via bootloader do Arduino (ajuste PORTA)
#   make clean

MCU    = atmega328p
F_CPU  = 16000000UL
PORTA ?= /dev/ttyACM0

CC      = avr-gcc
OBJCOPY = avr-objcopy
SIZE    = avr-size

SRC = src/main.c src/timer.c src/adc.c src/uart.c src/uart_fmt.c \
      src/params.c src/controle.c src/protocolo.c
OBJ = $(patsubst src/%.c,build/%.o,$(SRC))

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -std=gnu11 \
          -Wall -Wextra -Wshadow -Wstrict-prototypes \
          -ffunction-sections -fdata-sections -flto -g
LDFLAGS = -mmcu=$(MCU) -Wl,--gc-sections -flto -Os

all: build/nivel.hex size

build/%.o: src/%.c src/*.h | build
	$(CC) $(CFLAGS) -c $< -o $@

build/nivel.elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

build/nivel.hex: build/nivel.elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

build:
	mkdir -p build

size: build/nivel.elf
	$(SIZE) --format=avr --mcu=$(MCU) $<

flash: build/nivel.hex
	avrdude -p $(MCU) -c arduino -P $(PORTA) -b 115200 -U flash:w:$<:i

clean:
	rm -rf build

.PHONY: all size flash clean
