#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "softuart.h"
#include "light_ws2812.h"

#if defined(__AVR_ATmega8__)
  #define WS_PORT PORTC
  #define WS_DDR DDRC
  #define WS_POWER PC0
  #define WS_GROUND PC2
  #define WS_DATA PC1
#elif defined(__AVR_ATtiny85__)
#endif

#define START_PORT PORTB
#define START_PIN PINB
#define START_BIT PB6

#define BOUNCE_PORT PORTB
#define BOUNCE_PIN PINB
#define BOUNCE_BIT PB7

#define PING 'N'
#define SIG 'S'
#define ACK 'A'
#define SEL 'L'
#define MOVE 'M'
#define FLOOD 'F'
#define EMPTY 'E'

#define SELECTED 1

int8_t next = -1;
uint8_t status = 0;
uint8_t flood_mask = 0;
uint8_t neighbour_mask = 0b0000;

struct cRGB light = {0, 0, 0};

void empty (uint8_t n)
{
  if (!(flood_mask & _BV(n)))
    return;
  light.g = 0;
  ws2812_setleds(&light, 1);
  flood_mask &= ~(_BV(n));
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, EMPTY);
    softuart_putchar(i, n);
  }
}

void flood (uint8_t n)
{
  if (flood_mask & _BV(n))
    return;

  light.g = 100;
  ws2812_setleds(&light, 1);
  flood_mask |= _BV(n);
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, FLOOD);
    softuart_putchar(i, n);
  }
}

void select() {
  status = SELECTED;
  light.g = 100;
  ws2812_setleds(&light, 1);
  next = -1;
}

void move(uint8_t dir)
{
  // select adjacent node
  if (next == -1) {
    if (!(neighbour_mask & _BV(dir)))
      return;
    status = 0;
    light.g = 0;
    ws2812_setleds(&light, 1);

    softuart_putchar(dir, SEL);
    next = dir;

  // move along path
  } else {
    softuart_putchar(next, MOVE);
    softuart_putchar(next, dir);
  }
}

int main ()
{

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)
  WS_DDR |= _BV(WS_POWER) | _BV(WS_GROUND) | _BV(WS_DATA);
  WS_PORT |= _BV(WS_POWER); // ws2812 power set high

  // jumpers pulled high
  START_PORT |= _BV(START_BIT);
  BOUNCE_PORT |= _BV(BOUNCE_BIT);

  softuart_create_channel( CH(D, 0, 1) );
  softuart_create_channel( CH(D, 6, 7) );
  softuart_create_channel( CH(C, 4, 5) );
  softuart_create_channel( CH(B, 1, 2) );
#endif

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
    status = SELECTED;
    light.g = 100;
    ws2812_setleds(&light, 1);
  }

	while(1) {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)

    uint8_t bounce = !(BOUNCE_PIN & _BV(BOUNCE_BIT));

    for (uint8_t i = 0; i < 4; ++i) {

      if (!softuart_kbhit(i))
        continue;
      char c = softuart_getchar(i);
      switch(c) {

        case 'h': move(2); break;
        case 'j': move(0); break;
        case 'k': move(3); break;
        case 'l': move(1); break;

        case MOVE:
          move(softuart_getchar(i));
          break;

        case SEL:
          select();
          break;

        case FLOOD:
          flood(softuart_getchar(i) % 8);
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
}
#endif

