#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "softuart.h"

#ifdef __AVR_ATmega328P__
#endif
#ifdef __AVR_ATtiny85__
#endif

#define PING 'N'
#define PONG 'F'
#define LED_ON() ( PORTB |= _BV(PB4) )
#define LED_OFF() ( PORTB &= ~(_BV(PB4)) )
#define LED_FLASH() { LED_ON(); _delay_ms(500); LED_OFF(); }

int main ()
{

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)
  softuart_create_channel(
      &PIND, &DDRD, PD0,
      &PORTD,&DDRD, PD1);
  softuart_create_channel(
      &PIND, &DDRD, PD2,
      &PORTD,&DDRD, PD3);
#elif defined(__AVR_ATtiny85__)
  softuart_create_channel(
      &PINB, &DDRB, PB0,
      &PORTB,&DDRB, PB1);
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
