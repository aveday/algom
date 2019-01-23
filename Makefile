BAUD=19200
SOFTBAUD=2400
FREQ=8000000 # 8 Mhz
PROGRAMMER=usbasp
BOOTSTART=0x1800

CC=avr-gcc
CFLAGS=-DF_CPU=${FREQ} -DSOFTUART_BAUD_RATE=${SOFTBAUD} -mmcu=${MCU} -Wall -Werror -Wfatal-errors -Wextra -Os

SRC=${filter-out src/boot.c, ${wildcard src/*.c}}
OBJ=${SRC:src/%.c=obj/%.o}
DEP=${OBJ:.o=.d}

.PHONY: all flash clean

all: image.hex
	@echo ${MCU}
	@avr-size --mcu=${MCU} -C boot.elf
	@avr-size --mcu=${MCU} -C app.elf

app.elf: ${OBJ}
	@echo LD $@
	@${CC} ${CFLAGS} -o $@ $^

app.hex: app.elf
	@avr-objcopy -j .text -j .data -O ihex $^ $@

boot.elf: obj/boot.o
	@echo LD $@
	@${CC} ${CFLAGS} -Wl,--section-start=.text=${BOOTSTART} -o $@ $^

boot.hex: boot.elf
	@avr-objcopy -j .text -j .data -O ihex $^ $@

boot: boot.hex
	echo ${PROGRAMMER} ${MCU}
	@avrdude -p${MCU} -c${PROGRAMMER} -b${BAUD} -V -U flash:w:$<
	@avr-size --mcu=${MCU} -C boot.elf

image.hex: boot.hex app.hex
	srec_cat app.hex -I boot.hex -I -o image.hex -I

-include ${DEP}

obj/%.o: src/%.c
	@echo CC $@
	@mkdir -p obj
	@${CC} ${CFLAGS} -Isrc -MMD -MP -c $< -o $@

flash: image.hex
	echo ${PROGRAMMER} ${MCU}
	@avrdude -p${MCU} -c${PROGRAMMER} -b${BAUD} -V -U flash:w:$<

reset:
	@avrdude -p${MCU} -c${PROGRAMMER} -b${BAUD}

clean:
	@rm -rf obj *.elf *.hex
