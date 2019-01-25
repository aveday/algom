BAUD=19200
SOFTBAUD=2400
FREQ=8000000 # 8 Mhz
PROGRAMMER=usbasp
BOOTSTART=0x1800

CC=avr-gcc
CFLAGS=-DF_CPU=${FREQ} -DBOOTSTART=${BOOTSTART} -DSOFTUART_BAUD_RATE=${SOFTBAUD} -mmcu=${MCU} -Wall -Werror -Wfatal-errors -Wextra -Os

EXCLUDE=src/test.c src/boot.c
SRC=${filter-out ${EXCLUDE}, ${wildcard src/*.c}}
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

boot.elf: obj/boot.o obj/softuart.o
	@echo LD $@
	@${CC} ${CFLAGS} -Wl,--section-start=.text=${BOOTSTART} -o $@ $^

boot: boot.hex
	echo ${PROGRAMMER} ${MCU}
	@avrdude -p${MCU} -V -U flash:w:$<
	@avr-size --mcu=${MCU} -C boot.elf

image.hex: boot.hex app.hex
	srec_cat app.hex -I boot.hex -I -o image.hex -I

test.elf: obj/test.o
	@echo LD $@
	@${CC} ${CFLAGS} -o $@ $^

%.hex: %.elf
	@avr-objcopy -j .text -j .data -O ihex $^ $@

%.bin: %.hex
	@avr-objcopy -O binary --pad-to ${BOOTSTART} --gap-fill "0xFF" -I ihex $^ $@

-include ${DEP}

obj/%.o: src/%.c
	@echo CC $@
	@mkdir -p obj
	@${CC} ${CFLAGS} -Isrc -MMD -MP -c $< -o $@

flash: image.hex
	echo ${PROGRAMMER} ${MCU}
	@avrdude -p${MCU} -V -U flash:w:$<

reset:
	@avrdude -p${MCU}

clean:
	@rm -rf obj *.elf *.hex *.bin
