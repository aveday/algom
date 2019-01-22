#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "softuart.h"
#include "light_ws2812.h"

#define PING 'N'
#define PONG 'F'
#define SIG 'S'
#define ACK 'A'

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


void flash(uint8_t c) {
  struct cRGB light = {0, 0, 0};
  switch(c) {
    case 0: light.r = 255; break;
    case 1: light.g = 255; break;
    case 2: light.b = 255; break;
  }
  ws2812_setleds(&light, 1);
  _delay_ms(500);
  light.r = 0;
  light.g = 0;
  light.b = 0;
  ws2812_setleds(&light, 1);
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

#elif defined(__AVR_ATtiny85__)
  softuart_create_channel( RX(B,0), TX(B,1) );
#endif

  softuart_init();
  sei();

  _delay_ms(200);

  // send init signals to neighbours
  for (uint8_t i = 0; i < 4; ++i) {
    softuart_putchar(i, SIG);
  }
  flash(2);
  _delay_ms(200);

  if (!(START_PIN & _BV(START_BIT))) {
    flash(1);
    softuart_putchar(3, PING);
  }

  uint8_t neighbour_mask = 0b0000;
	while(1) {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)

    uint8_t bounce = !(BOUNCE_PIN & _BV(BOUNCE_BIT));

    for (uint8_t i = 0; i < 4; ++i) {

      if (!softuart_kbhit(i))
        continue;
      switch(softuart_getchar(i)) {
        case SIG:
          softuart_putchar(i, ACK);
          __attribute__ ((fallthrough));
        case ACK:
          neighbour_mask |= _BV(i);
          break;
        case PING:
          flash(0);
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

