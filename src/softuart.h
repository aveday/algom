#if !defined(F_CPU)
    #warning "F_CPU not defined in makefile - now defined in softuart.h"
    #define F_CPU 3686400UL
#endif

#define SOFTUART_BAUD_RATE      2400

#if defined (__AVR_ATtiny25__) || defined (__AVR_ATtiny45__) || defined (__AVR_ATtiny85__)
    #define SOFTUART_N 1
    #define SOFTUART_RX0PIN   PINB
    #define SOFTUART_RX0DDR   DDRB
    #define SOFTUART_RX0BIT   PB0

    #define SOFTUART_TX0PORT  PORTB
    #define SOFTUART_TX0DDR   DDRB
    #define SOFTUART_TX0BIT   PB1

    #define SOFTUART_RX1PIN   PINB
    #define SOFTUART_RX1DDR   DDRB
    #define SOFTUART_RX1BIT   PB0

    #define SOFTUART_TX1PORT  PORTB
    #define SOFTUART_TX1DDR   DDRB
    #define SOFTUART_TX1BIT   PB1

    #define SOFTUART_T_COMP_LABEL      TIM0_COMPA_vect
    #define SOFTUART_T_COMP_REG        OCR0A
    #define SOFTUART_T_CONTR_REGA      TCCR0A
    #define SOFTUART_T_CONTR_REGB      TCCR0B
    #define SOFTUART_T_CNT_REG         TCNT0
    #define SOFTUART_T_INTCTL_REG      TIMSK

    #define SOFTUART_CMPINT_EN_MASK    (1 << OCIE0A)

    #define SOFTUART_CTC_MASKA         (1 << WGM01)
    #define SOFTUART_CTC_MASKB         (0)

    /* "A timer interrupt must be set to interrupt at three times 
       the required baud rate." */
    #define SOFTUART_PRESCALE (8)
    // #define SOFTUART_PRESCALE (1)

    #if (SOFTUART_PRESCALE == 8)
        #define SOFTUART_PRESC_MASKA         (0)
        #define SOFTUART_PRESC_MASKB         (1 << CS01)
    #elif (SOFTUART_PRESCALE==1)
        #define SOFTUART_PRESC_MASKA         (0)
        #define SOFTUART_PRESC_MASKB         (1 << CS00)
    #else 
        #error "prescale unsupported"
    #endif
#elif defined (__AVR_ATmega324P__) || defined (__AVR_ATmega324A__)  \
   || defined (__AVR_ATmega644P__) || defined (__AVR_ATmega644PA__) \
   || defined (__AVR_ATmega328P__) || defined (__AVR_ATmega328PA__) \
   || defined (__AVR_ATmega164P__) || defined (__AVR_ATmega164A__)

    #define SOFTUART_RX0PIN   PIND
    #define SOFTUART_RX0DDR   DDRD
    #define SOFTUART_RX0BIT   PD0

    #define SOFTUART_TX0PORT  PORTD
    #define SOFTUART_TX0DDR   DDRD
    #define SOFTUART_TX0BIT   PD1

    #define SOFTUART_T_COMP_LABEL      TIMER0_COMPA_vect
    #define SOFTUART_T_COMP_REG        OCR0A
    #define SOFTUART_T_CONTR_REGA      TCCR0A
    #define SOFTUART_T_CONTR_REGB      TCCR0B
    #define SOFTUART_T_CNT_REG         TCNT0
    #define SOFTUART_T_INTCTL_REG      TIMSK0
    #define SOFTUART_CMPINT_EN_MASK    (1 << OCIE0A)
    #define SOFTUART_CTC_MASKA         (1 << WGM01)
    #define SOFTUART_CTC_MASKB         (0)

    /* "A timer interrupt must be set to interrupt at three times 
       the required baud rate." */
    #define SOFTUART_PRESCALE (8)
    // #define SOFTUART_PRESCALE (1)

    #if (SOFTUART_PRESCALE == 8)
        #define SOFTUART_PRESC_MASKA         (0)
        #define SOFTUART_PRESC_MASKB         (1 << CS01)
    #elif (SOFTUART_PRESCALE==1)
        #define SOFTUART_PRESC_MASKA         (0)
        #define SOFTUART_PRESC_MASKB         (1 << CS00)
    #else 
        #error "prescale unsupported"
    #endif
#elif defined (__AVR_ATmega8__)
    #define SOFTUART_N 2

    #define SOFTUART_RX0PIN   PIND
    #define SOFTUART_RX0DDR   DDRD
    #define SOFTUART_RX0BIT   PD0

    #define SOFTUART_TX0PORT  PORTD
    #define SOFTUART_TX0DDR   DDRD
    #define SOFTUART_TX0BIT   PD1

    #define SOFTUART_RX1PIN   PIND
    #define SOFTUART_RX1DDR   DDRD
    #define SOFTUART_RX1BIT   PD2

    #define SOFTUART_TX1PORT  PORTD
    #define SOFTUART_TX1DDR   DDRD
    #define SOFTUART_TX1BIT   PD3

    #define SOFTUART_T_COMP_LABEL      TIMER1_COMPA_vect
    #define SOFTUART_T_COMP_REG        OCR1A
    #define SOFTUART_T_CONTR_REGA      TCCR1A
    #define SOFTUART_T_CONTR_REGB      TCCR1B
    #define SOFTUART_T_CNT_REG         TCNT1
    #define SOFTUART_T_INTCTL_REG      TIMSK
    #define SOFTUART_CMPINT_EN_MASK    (1 << OCIE1A)
    #define SOFTUART_CTC_MASKA         (0)
    #define SOFTUART_CTC_MASKB         (1 << WGM12)

    /* "A timer interrupt must be set to interrupt at three times 
       the required baud rate." */
    #define SOFTUART_PRESCALE (8)
    // #define SOFTUART_PRESCALE (1)

    #if (SOFTUART_PRESCALE == 8)
        #define SOFTUART_PRESC_MASKA         (0)
        #define SOFTUART_PRESC_MASKB         (1 << CS11)
    #elif (SOFTUART_PRESCALE==1)
        #define SOFTUART_PRESC_MASKA         (0)
        #define SOFTUART_PRESC_MASKB         (1 << CS10)
    #else 
        #error "prescale unsupported"
    #endif
#else
    #error "no defintions available for this AVR"
#endif

#define SOFTUART_TIMERTOP ( F_CPU/SOFTUART_PRESCALE/SOFTUART_BAUD_RATE/3 - 1)

#if (SOFTUART_TIMERTOP > 0xff)
    #warning "Check SOFTUART_TIMERTOP: increase prescaler, lower F_CPU or use a 16 bit timer"
#endif

#define SOFTUART_IN_BUF_SIZE     32

// Init the Software Uart
void softuart_init(unsigned char);

// Clears the contents of the input buffer.
void softuart_flush_input_buffer( unsigned char );

// Tests whether an input character has been received.
unsigned char softuart_kbhit( unsigned char );

// Reads a character from the input buffer, waiting if necessary.
char softuart_getchar( unsigned char );

// To check if transmitter is busy
unsigned char softuart_transmit_busy( unsigned char );

// Writes a character to the serial port.
void softuart_putchar( unsigned char, const char );

// Turns on the receive function.
void softuart_turn_rx_on( unsigned char );

// Turns off the receive function.
void softuart_turn_rx_off( unsigned char );

// Write a NULL-terminated string from RAM to the serial port
void softuart_puts( unsigned char, const char *s );

// Write a NULL-terminated string from program-space (flash) 
// to the serial port. example: softuart_puts_p(PSTR("test"))
void softuart_puts_p( unsigned char, const char *prg_s );

// Helper-Macro - "automatically" inserts PSTR
// when used: include avr/pgmspace.h before this include-file
#define softuart_puts_P(s___) softuart_puts_p(PSTR(s___))
