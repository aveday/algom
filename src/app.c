#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "softuart.h"
#include "light_ws2812.h"
#include "signals.h"

#define WS_PORT PORTC
#define WS_DDR DDRC
#define WS_POWER PC0
#define WS_GROUND PC2
#define WS_DATA PC1

#define START_PORT PORTB
#define START_PIN PINB
#define START_BIT PB6

#define BOUNCE_PORT PORTB
#define BOUNCE_PIN PINB
#define BOUNCE_BIT PB7

#define ASCII_NUM_START   0x30

// status macros
#define SELECTED 0
#define PREVSET  1
uint8_t status = 0;

// dirmask macros
#define DIRMASK  0b10000000
#define RELAY    0b00100000
#define SELF     0b00010000

// neighbours
uint8_t next, prev;
uint8_t neighbour_mask = 0b0000;
uint8_t clone_mask = 0b0000;

struct cRGB light = {0, 0, 0};

static uint8_t flash_speed = 250;
void set_flash (uint8_t speed) 
{
  TCCR0 = speed ? _BV(CS02) | _BV(CS00) : 0;
  flash_speed = speed;
}

ISR(TIMER0_OVF_vect)
{
  static uint8_t  c = 0;
  if (c--) return;
  c = ~flash_speed;
  light.r = ~light.r;
  ws2812_setleds(&light, 1);
}

void flood (int8_t src)
{
  if (status & _BV(PREVSET))
    return;

  prev = src;
  status |= _BV(PREVSET);
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, FLOOD);
  }
}

void boot(uint8_t src)
{
  if (clone_mask & _BV(src)) {
    softuart_putchar(src, PROGRAM_AVAILABLE);
    clone_mask &= ~(_BV(src));
  }
}

void reset()
{
  wdt_enable(WDTO_250MS);
  set_flash(254);
  for(;;) {}
}

void clone(uint8_t src)
{
  softuart_putchar(src, PROGRAM_GET);
  reset();
}

void select(uint8_t src)
{
  status |= _BV(SELECTED);
  light.r = 30;
  light.b = 30;
  ws2812_setleds(&light, 1);
  softuart_putchar(prev, BACK);
  softuart_putchar(src, DESEL);
}

void deselect()
{
  status &= ~(_BV(SELECTED));
  light.r = 0;
  light.b = 0;
  ws2812_setleds(&light, 1);
}

void backtrack(uint8_t src)
{
  next = src;
  softuart_putchar(prev, BACK);
}

void send_page(uint8_t dir, uint16_t page) {
  // check for program end
  if (page >= BOOTSTART) {
    softuart_putchar(dir, PROGRAM_END);
    return;
  }

  // check for page contents
  uint8_t page_empty = 1;
  uint16_t addr = page;
  for (uint16_t i = 0; i < SPM_PAGESIZE; ++i)
    if (pgm_read_byte(addr++) != 0xFF)
      page_empty = 0;
  if (page_empty) {
    softuart_putchar(dir, PAGE_EMPTY);
    return;
  }

  addr = page;
  softuart_putchar(dir, PAGE_SEND);
  for (uint16_t i = 0; i < SPM_PAGESIZE; ++i)
    softuart_putchar(dir, pgm_read_byte(addr++));
}

int main ()
{
  wdt_reset();

  WS_DDR |= _BV(WS_POWER) | _BV(WS_GROUND) | _BV(WS_DATA);
  WS_PORT |= _BV(WS_POWER); // ws2812 power set high

  // jumpers pulled high
  START_PORT |= _BV(START_BIT);
  BOUNCE_PORT |= _BV(BOUNCE_BIT);

  softuart_create_channel( CH(D, 0, 1) );
  softuart_create_channel( CH(D, 6, 7) );
  softuart_create_channel( CH(C, 4, 5) );
  softuart_create_channel( CH(B, 1, 2) );

  softuart_init();
  sei();

  _delay_ms(200);

  // send init signals to neighbours
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, SIG);
  }

  // startup flash
  light.b = 100;
  ws2812_setleds(&light, 1);
  light.b = 0;
  _delay_ms(200);
  ws2812_setleds(&light, 1);

  if (!(START_PIN & _BV(START_BIT))) {
    // floodfill cells to set backtracking direction
    flood(0);
    select(0);
  }

  // init flash timer
  TIMSK |= _BV(TOIE0); // enable timer0 overflow interrupt

  uint16_t addr[4];
  uint8_t dirmask[4] = {
    DIRMASK | SELF,
    DIRMASK | SELF,
    DIRMASK | SELF,
    DIRMASK | SELF
  };

  wdt_enable(WDTO_1S);
  // loop forever over uart channels
  for (uint8_t i = 0;; i = (i + 1) % 4) {
    wdt_reset();
    uint8_t bounce = !(BOUNCE_PIN & _BV(BOUNCE_BIT));

    if (!softuart_kbhit(i)) continue;
    uint8_t cmd = softuart_getchar(i);

    // check if dirmask
    if (cmd & DIRMASK) {
      dirmask[i] = cmd;
      continue;
    }

    // generate appropriate dirmask from user commands
    switch (cmd) {
      case 'h': cmd = SEL,   dirmask[i] = DIRMASK|RELAY|0b0100; break;
      case 'j': cmd = SEL,   dirmask[i] = DIRMASK|RELAY|0b0001; break;
      case 'k': cmd = SEL,   dirmask[i] = DIRMASK|RELAY|0b1000; break;
      case 'l': cmd = SEL,   dirmask[i] = DIRMASK|RELAY|0b0010; break;

      case 'H': cmd = CLONE, dirmask[i] = DIRMASK|RELAY|0b0100; break;
      case 'J': cmd = CLONE, dirmask[i] = DIRMASK|RELAY|0b0001; break;
      case 'K': cmd = CLONE, dirmask[i] = DIRMASK|RELAY|0b1000; break;
      case 'L': cmd = CLONE, dirmask[i] = DIRMASK|RELAY|0b0010; break;
    }

    // --- Relaying ---

    if (dirmask[i] & RELAY) {
      if (status & _BV(SELECTED)) {
        // remove relay mask if node is selected
        dirmask[i] &= ~RELAY;
      } else {
        // otherwise relay command and go to next message
        softuart_putchar(next, dirmask[i]);
        softuart_putchar(next, cmd);
        continue;
      }
    }

    // --- Not relaying ---

    // command neighbours
    for (uint8_t d = 0; d < 4; ++d) {
      if ((dirmask[i] >> d) & 1) {
        softuart_putchar(d, DIRMASK | SELF);
        softuart_putchar(d, cmd);
      }
    }

    // command self
    if (dirmask[i] & SELF) switch(cmd) {
      case BOOT:    boot(i);        break;
      case CLONE:   clone(i);       break;
      case SEL:     select(i);      break;
      case DESEL:   deselect();     break;
      case BACK:    backtrack(i);   break;
      case FLOOD:   flood(i);       break;
      case RESET:   reset();        break;

      case PROGRAM_GET:
        addr[i] = 0x00;
        clone_mask |= _BV(i);
        break;

      case PAGE_GET:
        send_page(i, addr[i]);
        addr[i] += SPM_PAGESIZE;
        break;

      case SIG:
        softuart_putchar(i, ACK);
        __attribute__ ((fallthrough));
      case ACK:
        neighbour_mask |= _BV(i);
        break;
      case PING:
        light.r =  ~light.r;
        ws2812_setleds(&light, 1);
        uint8_t n = 1;
        for (; n < 4; ++n)
          if ( _BV((i+n) % 4) & neighbour_mask )
            break;
        softuart_putchar(bounce ? i : (i + n) % 4, PING);
        break;
    }

    // reset dirmask to default (self)
    dirmask[i] = DIRMASK | SELF;
  }
}

