#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/wdt.h>
#include <inttypes.h>
#include <util/delay.h>
#include "softuart.h"
#include "signals.h"

#define FLASH_APP_START_ADDR 0x0000
#define NONE -1

// jump to main application
typedef void (*fptr_t)(void);
fptr_t start_app = (fptr_t)0x0000;

#define ASCII_NUM_START   0x30
#define ASCII_ALPHA_START 0x41

#define CHAR_TO_NYBBLE(ch) ( \
  (ch) < ASCII_ALPHA_START \
    ? (ch) - ASCII_NUM_START \
    : (ch) - ASCII_ALPHA_START + 0xA \
)

#define NYBBLE_TO_CHAR(nybble) ( \
  (nybble) < 0xA \
    ? (nybble) + ASCII_NUM_START \
    : (nybble) + ASCII_ALPHA_START - 0xA \
)

void byte_to_string(char* str, uint8_t byte) {
  str[0] = NYBBLE_TO_CHAR(byte >> 0x4);
  str[1] = NYBBLE_TO_CHAR(byte & 0xF);
}

uint8_t page_buffer[SPM_PAGESIZE];
void boot_program_page (uint32_t page, uint8_t *buf)
{

  for (uint16_t i=0; i<SPM_PAGESIZE; i+=2) {
    // Set up little-endian word.
    uint16_t w = *buf++;
    w += (*buf++) << 8;
    boot_page_fill_safe (page + i, w);
  }
  cli();
  boot_page_erase_safe (page);
  boot_page_write_safe (page);
  sei();
}

int main() {
  wdt_enable(WDTO_1S);

  // set interrupt vectors to bootloader
  GICR = _BV(IVCE);
  GICR = _BV(IVSEL);

  // setup softuart
  softuart_create_channel( CH(D, 0, 1) );
  softuart_create_channel( CH(D, 6, 7) );
  softuart_create_channel( CH(C, 4, 5) );
  softuart_create_channel( CH(B, 1, 2) );
  softuart_init();
  sei();
  _delay_ms(10);

  DDRB |= _BV(PB0);

  // send out boot signal
  for (int i = 0; i < 4; ++i)
    softuart_putchar(i, BOOT);
  _delay_ms(10);

  wdt_reset();

  // check for available program
  int8_t sender = NONE;
  for (uint8_t i= 0; i< 4; i++) {
    if (softuart_kbhit(i) && softuart_getchar(i) == PROGRAM_AVAILABLE) {
      sender = i;
      break;
    }
  }

  wdt_reset();

  // receive program from sender
  if (sender != NONE) {
    PORTB |= _BV(PB0);

    uint32_t page = FLASH_APP_START_ADDR;

    while (1) {
      wdt_reset();

      // ask for a program page from the sender
      softuart_putchar(sender, PAGE_GET);
      uint8_t response = softuart_getchar(sender);

      if (response == PROGRAM_END) {
        break;

      } else if (response == PAGE_EMPTY) {
        cli();
        boot_page_erase_safe (page);
        sei();

      } else if (response == PAGE_SEND) {
        // read program bytes into page buffer
        for (uint16_t i = 0; i < SPM_PAGESIZE; ++i)
          page_buffer[i] = softuart_getchar(sender);

        // then program page
        boot_program_page(page, page_buffer);
        page += SPM_PAGESIZE;
      }
    }
    PORTB &= ~(_BV(PB0));
  }

  // Reenable RWW-section again
  boot_rww_enable_safe ();

  wdt_reset();

  // return interrupt vectors to application
  GICR = _BV(IVCE);
  GICR = 0;
  start_app();
}
