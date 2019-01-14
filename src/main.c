#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "softuart.h"

#define PING 'N'
#define PONG 'F'
#define LED_ON() ( PORTB |= _BV(PB4) )
#define LED_OFF() ( PORTB &= ~(_BV(PB4)) )
#define LED_FLASH() { LED_ON(); _delay_ms(500); LED_OFF(); }

int main ()
{

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)
  softuart_create_channel( RX(D,0), TX(D,1) );
  softuart_create_channel( RX(D,2), TX(D,3) );
#elif defined(__AVR_ATtiny85__)
  softuart_create_channel( RX(B,0), TX(B,1) );
#endif

  softuart_init();

  DDRB |= _BV(PB4); // LED pin
  sei();

  _delay_ms(500);
  softuart_putchar(0, PING);

	while(1) {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)

    if (softuart_kbhit(0)) {
      char b = softuart_getchar(0);
      if (b == PONG) {
        LED_FLASH();
        softuart_putchar(1, PING);
      }
    }

    else if (softuart_kbhit(1)) {
      char b = softuart_getchar(1);
      if (b == PONG) {
        LED_FLASH();
        softuart_putchar(0, PING);
      }
    }

#elif defined (__AVR_ATtiny85__)
    if (softuart_kbhit(0)) {
      char b = softuart_getchar(0);
      if (b == PING) {
        LED_FLASH();
        softuart_putchar(0, PONG);
      }
    }
#endif
	}
}
