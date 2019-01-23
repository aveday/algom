#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

typedef void (*fptr_t)(void);
fptr_t load_app = (fptr_t)0x0000;

int main() {
  cli();

  DDRB |= _BV(PB0);
  uint8_t i = 10;
  while (--i) {
    _delay_ms(200);
    PORTB ^= _BV(PB0);
  }

  PORTB = 0;
  DDRB = 0;

  load_app();
}
