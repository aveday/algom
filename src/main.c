#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "softuart.h"

#ifdef __AVR_ATmega328P__
#endif
#ifdef __AVR_ATtiny85__
#endif

#define SIG0 'N'
#define SIG1 'F'
#define LED_ON() ( PORTB |= _BV(PB4) )
#define LED_OFF() ( PORTB &= ~(_BV(PB4)) )

int main ()
{
  softuart_init();
  DDRB |= _BV(PB4); // LED pin
  softuart_turn_rx_on();
  sei();

	while(1) {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega8__)
    while (softuart_transmit_busy());
    softuart_putchar(SIG0);
    LED_ON();

    _delay_ms(1000);

    while (softuart_transmit_busy());
    LED_OFF();
    softuart_putchar(SIG1);

    _delay_ms(1000);
#elif defined (__AVR_ATtiny85__)
    char b = softuart_getchar();
    if (b == SIG0) LED_ON();
    if (b == SIG1) LED_OFF();
#endif
	}
}
