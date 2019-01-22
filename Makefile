BAUD=19200
SOFTBAUD=2400
FREQ=8000000 # 8 Mhz
PROGRAMMER=usbasp

CC=avr-gcc
CFLAGS=-DF_CPU=${FREQ} -DSOFTUART_BAUD_RATE=${SOFTBAUD} -mmcu=${MCU} -Wall -Werror -Wfatal-errors -Wextra -Os

SRC=${wildcard src/*.c}
OBJ=${SRC:src/%.c=obj/%.o}
DEP=${OBJ:.o=.d}

.PHONY: all flash clean

all: main.hex
	@echo ${MCU}
	@avr-size --mcu=${MCU} -C main.elf

main.elf: ${OBJ}
	@echo LD $@
	@${CC} ${CFLAGS} -o $@ $^

main.hex: main.elf
	@avr-objcopy -j .text -j .data -O ihex $^ $@

-include ${DEP}

obj/%.o: src/%.c
	@echo CC $@
	@mkdir -p obj
	@${CC} ${CFLAGS} -Isrc -MMD -MP -c $< -o $@

flash: main.hex
	echo ${PROGRAMMER} ${MCU}
	@avrdude -p${MCU} -c${PROGRAMMER} -b${BAUD} -V -U flash:w:$<
	@avr-size --mcu=${MCU} -C main.elf

clean:
	@rm -rf obj *.elf *.hex
