#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "softuart.h"
#include "light_ws2812.h"

#define PING 'N'
#define PONG 'F'

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
#define START_BIT PB1

#define BOUNCE_PORT PORTB
#define BOUNCE_PIN PINB
#define BOUNCE_BIT PB2

struct cRGB light = {0, 0, 0};

void flash() {
  light.r = 255;
  ws2812_setleds(&light, 1);
  _delay_ms(500);
  light.r = 0;
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

  softuart_create_channel( RX(D,0), TX(D,1) );
  softuart_create_channel( RX(D,6), TX(D,7) );
#elif defined(__AVR_ATtiny85__)
  softuart_create_channel( RX(B,0), TX(B,1) );
#endif

  softuart_init();
  sei();

  _delay_ms(500);
  if (!(START_PIN & _BV(START_BIT))) {
    flash();
    softuart_putchar(0, PING);
  }

	while(1) {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)

    uint8_t bounce = !(BOUNCE_PIN & _BV(BOUNCE_BIT));

    if (softuart_kbhit(0)) {
      char b = softuart_getchar(0);
      if (b == PING) {
        flash();
        softuart_putchar(bounce ? 0 : 1, PING);
      }
    }

    else if (softuart_kbhit(1)) {
      char b = softuart_getchar(1);
      if (b == PING) {
        flash();
        softuart_putchar(bounce ? 1 : 0, PING);
      }
    }
  }
}
#endif

