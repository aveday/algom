#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
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

// status macros
#define SELECTED 0
#define PREVSET  1
uint8_t status = 0;

// neighbours
uint8_t next, prev;
uint8_t neighbour_mask = 0b0000;

struct cRGB light = {0, 0, 0};

void empty (uint8_t n)
{
  if (!(status & _BV(n)))
    return;
  status &= ~(_BV(n));
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, EMPTY);
    softuart_putchar(i, n);
  }
}

void flood (int8_t src, uint8_t n)
{
  if (status & _BV(n))
    return;

  prev = src;
  status |= _BV(n);
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, FLOOD);
    softuart_putchar(i, n);
  }
}

void select()
{
  status |= _BV(SELECTED);
  light.g = 30;
  ws2812_setleds(&light, 1);
  softuart_putchar(prev, BACK);
}

void backtrack(uint8_t src)
{
  next = src;
  softuart_putchar(prev, BACK);
}

void move(uint8_t dir)
{
  // select adjacent node
  if (status & _BV(SELECTED)) {
    // prevent falling off edge of world
    if (!(neighbour_mask & _BV(dir)))
      return;

    status &= ~(_BV(SELECTED));
    light.g = 0;
    ws2812_setleds(&light, 1);

    next = dir;
    softuart_putchar(dir, SEL);

  // move along path
  } else {
    softuart_putchar(next, MOVE);
    softuart_putchar(next, dir);
  }
}

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
  light.b = ~light.b;
  ws2812_setleds(&light, 1);
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
    flood(0, PREVSET);
    select();
  }

  // init flash timer
  TIMSK |= _BV(TOIE0); // enable timer0 overflow interrupt
  //set_flash(225);

  uint16_t addr[4];

  // loop forever over uart channels
  for (uint8_t i = 0;; i = (i + 1) % 4) {
    uint8_t bounce = !(BOUNCE_PIN & _BV(BOUNCE_BIT));

    if (!softuart_kbhit(i))
      continue;
    char c = softuart_getchar(i);
    switch(c) {

      case 'h': move(2); break;
      case 'j': move(0); break;
      case 'k': move(3); break;
      case 'l': move(1); break;

      case BOOT:
        // XXX only offer program south
        if (i == 0)
          softuart_putchar(i, PROGRAM_AVAILABLE);
        break;

      case PROGRAM_GET:
        addr[i] = 0x00;
        break;

      case PAGE_GET:
        send_page(i, addr[i]);
        addr[i] += SPM_PAGESIZE;
        break;

      case MOVE:
        move(softuart_getchar(i));
        break;

      case SEL:
        select();
        break;

      case BACK:
        backtrack(i);
        break;

      case FLOOD:
        flood(i, softuart_getchar(i) % 8);
        break;
      case EMPTY:
        empty(softuart_getchar(i) % 8);
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
  }
}

