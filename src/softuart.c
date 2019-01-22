// softuart.c
// AVR-port of the generic software uart written in C
//
// Generic code from
// Colin Gittins, Software Engineer, Halliburton Energy Services
// (has been available from iar.com web-site -> application notes)
//
// Adapted to AVR using avr-gcc and avr-libc
// by Martin Thomas, Kaiserslautern, Germany
// <eversmith@heizung-thomas.de> 
// http://www.siwawi.arubi.uni-kl.de/avr_projects
//
// AVR-port Version 0.4  10/2010
//
// ---------------------------------------------------------------------
//
// Remarks from Colin Gittins:
//
// Generic software uart written in C, requiring a timer set to 3 times
// the baud rate, and two software read/write pins for the receive and
// transmit functions.
//
// * Received characters are buffered
// * putchar(), getchar(), kbhit() and flush_input_buffer() are available
// * There is a facility for background processing while waiting for input
// The baud rate can be configured by changing the BAUD_RATE macro as
// follows:
//
// #define BAUD_RATE  19200.0
//
// The function init_uart() must be called before any comms can take place
//
// Interface routines required:
// 1. get_rx_pin_status()
//    Returns 0 or 1 dependent on whether the receive pin is high or low.
// 2. set_tx_pin_high()
//    Sets the transmit pin to the high state.
// 3. set_tx_pin_low()
//    Sets the transmit pin to the low state.
// 4. idle()
//    Background functions to execute while waiting for input.
// 5. timer_set( BAUD_RATE )
//    Sets the timer to 3 times the baud rate.
// 6. set_timer_interrupt( timer_isr )
//    Enables the timer interrupt.
//
// Functions provided:
// 1. void flush_input_buffer( void )
//    Clears the contents of the input buffer.
// 2. char kbhit( void )
//    Tests whether an input character has been received.
// 3. char getchar( void )
//    Reads a character from the input buffer, waiting if necessary.
// 4. void turn_rx_on( void )
//    Turns on the receive function.
// 5. void turn_rx_off( void )
//    Turns off the receive function.
// 6. void putchar( char )
//    Writes a character to the serial port.
//
// ---------------------------------------------------------------------

/* 
Remarks by Martin Thomas (avr-gcc/avr-libc):
V0.1 (2/2005)
- stdio.h not used
- AVR-Timer in CTC-Mode ("manual" reload may not be accurate enough)
- Global Interrupt Flag has to be enabled (see demo-application)
- Interface timer_set and set_timer_interrupt not used here
- internal_tx_buffer was defined as unsigned char - thas could not
  work since more than 8 bits are needed, changed to unsigned short
- some variables moved from "global scope" into ISR function-scope
- GPIO initialisation included
- Added functions for string-output inspired by P. Fleury's AVR UART-lib.
V0.2 (3/2007)
- adjusted number of RX-bits
- adapted to avr-libc ISR-macro (replaces SIGNAL)
- disable interrupts during timer-init
- used unsigned char (uint8_t) where apropriate
- removed "magic" char checking (0xc2)
- added softuart_can_transmit()
- Makefile based on template from WinAVR 1/2007
- reformated
- extended demo-application to show various methods to 
  send a string from flash and RAM
- demonstrate usage of avr-libc's stdio in demo-applcation
- tested with ATmega644 @ 3,6864MHz system-clock using
  avr-gcc 4.1.1/avr-libc 1.4.5 (WinAVR 1/2007)
V0.3 (4/2007)
- better configuration options in softuart.h.
  ->should be easier to adapt to different AVRs
- tested with ATtiny85 @ 1MHz (internal RC) with 2400bps
- AVR-Studio Project-File
V0.4 (10/2010)
- added options for ATmega164P, ATmega32P, ATmega64P
- changed some variable-types from char to unsigned char
- changed some comparisons from <= to ==
- small optimization in ISR for RX with temporary variable
- minor modifications in comments and formating
- added compiler options -fno-inline-small-functions, -Wl,--relax
- renamed flag_tx_ready to flag_tx_busy
- replaced softuart_can_transmit() by softuart_transmit_busy()
- tested with ATmega324PV @ 1MHz internal RC and 2400bps
  (options for ATtiny25/45/85 still available)
- added 3BSD license
- removed redundant zero-init in declaration of qin and qout
*/

/* Copyright (c) 2003, Colin Gittins
   Copyright (c) 2005, 2007, 2010, Martin Thomas
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "softuart.h"

#define SU_TRUE    1
#define SU_FALSE   0

// startbit and stopbit parsed internally (see ISR)
#define RX_NUM_OF_BITS (8)
static volatile char           inbuf[SOFTUART_LIMIT][SOFTUART_IN_BUF_SIZE];
static volatile unsigned char  qin[SOFTUART_LIMIT];
static unsigned char           qout[SOFTUART_LIMIT];
static volatile unsigned char  flag_rx_off[SOFTUART_LIMIT];
static volatile unsigned char  flag_rx_ready[SOFTUART_LIMIT];
static volatile unsigned char  flag_rx_off[SOFTUART_LIMIT];
static volatile unsigned char  flag_rx_ready[SOFTUART_LIMIT];

// 1 Startbit, 8 Databits, 1 Stopbit = 10 Bits/Frame
#define TX_NUM_OF_BITS (10)
static volatile unsigned char  flag_tx_busy[SOFTUART_LIMIT];
static volatile unsigned char  timer_tx_ctr[SOFTUART_LIMIT];
static volatile unsigned char  bits_left_in_tx[SOFTUART_LIMIT];
static volatile unsigned short internal_tx_buffer[SOFTUART_LIMIT]; /* ! mt: was type uchar - this was wrong */
static uint8_t softuart_channels = 0;

volatile uint8_t *softuart_ddr[SOFTUART_LIMIT];
volatile uint8_t *softuart_pin[SOFTUART_LIMIT];
volatile uint8_t *softuart_port[SOFTUART_LIMIT];
uint8_t softuart_rxbit[SOFTUART_LIMIT];
uint8_t softuart_txbit[SOFTUART_LIMIT];

void set_tx_pin_high(unsigned char i) {
  *softuart_port[i] |=  ( 1 << softuart_txbit[i] );
}

void set_tx_pin_low(unsigned char i) {
  *softuart_port[i] &=  ~( 1 << softuart_txbit[i] );
}

unsigned char get_rx_pin_status(unsigned char i) {
  return *softuart_pin[i] & ( 1 << softuart_rxbit[i] );
}

ISR(SOFTUART_T_COMP_LABEL)
{
	static unsigned char flag_rx_waiting_for_stop_bit[SOFTUART_LIMIT];
	static unsigned char rx_mask[SOFTUART_LIMIT];
	
	static unsigned char timer_rx_ctr[SOFTUART_LIMIT];
	static unsigned char bits_left_in_rx[SOFTUART_LIMIT];
	static unsigned char internal_rx_buffer[SOFTUART_LIMIT];
	
	unsigned char start_bit[SOFTUART_LIMIT], flag_in[SOFTUART_LIMIT];
	unsigned char tmp[SOFTUART_LIMIT];
	
	// Transmitter Section
  for (unsigned char i = 0; i < softuart_channels; ++i) {
    if ( flag_tx_busy[i] == SU_TRUE ) {
      tmp[i] = timer_tx_ctr[i];
      if ( --(tmp[i]) == 0 ) { // if ( --timer_tx_ctr <= 0 )
        if ( internal_tx_buffer[i] & 0x01 ) {
          set_tx_pin_high(i);
        }
        else {
          set_tx_pin_low(i);
        }
        internal_tx_buffer[i] >>= 1;
        tmp[i] = 3; // timer_tx_ctr = 3;
        if ( --(bits_left_in_tx[i]) == 0 ) {
          flag_tx_busy[i] = SU_FALSE;
        }
      }
      timer_tx_ctr[i] = tmp[i];
    }

    // Receiver Section
    if ( flag_rx_off[i] == SU_FALSE ) {
      if ( flag_rx_waiting_for_stop_bit[i] ) {
        if ( --(timer_rx_ctr[i]) == 0 ) {
          flag_rx_waiting_for_stop_bit[i] = SU_FALSE;
          flag_rx_ready[i] = SU_FALSE;
          inbuf[i][qin[i]] = internal_rx_buffer[i];
          if ( ++(qin[i]) >= SOFTUART_IN_BUF_SIZE ) {
            // overflow - reset inbuf-index
            qin[i] = 0;
          }
        }
      }
      else {  // rx_test_busy
        if ( flag_rx_ready[i] == SU_FALSE ) {
          start_bit[i] = get_rx_pin_status(i);
          // test for start bit
          if ( start_bit[i] == 0 ) {
            flag_rx_ready[i]      = SU_TRUE;
            internal_rx_buffer[i] = 0;
            timer_rx_ctr[i]       = 4;
            bits_left_in_rx[i]    = RX_NUM_OF_BITS;
            rx_mask[i]            = 1;
          }
        }
        else {  // rx_busy
          tmp[i] = timer_rx_ctr[i];
          if ( --(tmp[i]) == 0 ) { // if ( --timer_rx_ctr == 0 ) {
            // rcv
            tmp[i] = 3;
            flag_in[i] = get_rx_pin_status(i);
            if ( flag_in[i] ) {
              internal_rx_buffer[i] |= rx_mask[i];
            }
            rx_mask[i] <<= 1;
            if ( --(bits_left_in_rx[i]) == 0 ) {
              flag_rx_waiting_for_stop_bit[i] = SU_TRUE;
            }
          }
          timer_rx_ctr[i] = tmp[i];
        }
      }
    }
  }
}

static void io_init(unsigned char i)
{
  // TX-Pin as output
  *softuart_ddr[i] |=  ( 1 << softuart_txbit[i] );
  // RX-Pin as input
  *softuart_ddr[i] &= ~( 1 << softuart_rxbit[i] );
}

static void timer_init(void)
{
	unsigned char sreg_tmp;
	
	sreg_tmp = SREG;
	cli();
	
	SOFTUART_T_COMP_REG = SOFTUART_TIMERTOP;     /* set top */

	SOFTUART_T_CONTR_REGA = SOFTUART_CTC_MASKA | SOFTUART_PRESC_MASKA;
	SOFTUART_T_CONTR_REGB = SOFTUART_CTC_MASKB | SOFTUART_PRESC_MASKB;

	SOFTUART_T_INTCTL_REG |= SOFTUART_CMPINT_EN_MASK;

	SOFTUART_T_CNT_REG = 0; /* reset counter */
	
	SREG = sreg_tmp;
}

uint8_t softuart_create_channel(
    volatile uint8_t *ddr,
    volatile uint8_t *pin,
    volatile uint8_t *port,
    uint8_t rxbit, uint8_t txbit) {

  softuart_ddr[softuart_channels] = ddr;
  softuart_pin[softuart_channels] = pin;
  softuart_port[softuart_channels] = port;
  softuart_rxbit[softuart_channels] = rxbit;
  softuart_txbit[softuart_channels] = txbit;
  return ++softuart_channels;
}

void softuart_init( void )
{
  for (unsigned char i = 0; i < softuart_channels; ++i) {
    flag_tx_busy[i]  = SU_FALSE;
    flag_rx_ready[i] = SU_FALSE;
    flag_rx_off[i]   = SU_FALSE;
    
    set_tx_pin_high(i); /* mt: set to high to avoid garbage on init */

    io_init(i);
  }
	timer_init();
}

static void idle(void)
{
	// timeout handling goes here 
	// - but there is a "softuart_kbhit" in this code...
	// add watchdog-reset here if needed
}

void softuart_turn_rx_on( unsigned char i )
{
	flag_rx_off[i] = SU_FALSE;
}

void softuart_turn_rx_off( unsigned char i )
{
	flag_rx_off[i] = SU_TRUE;
}

char softuart_getchar( unsigned char i )
{
	char ch;

	while ( qout[i] == qin[i] ) {
		idle();
	}
	ch = inbuf[i][qout[i]];
	if ( ++(qout[i]) >= SOFTUART_IN_BUF_SIZE ) {
		qout[i] = 0;
	}
	
	return( ch );
}

unsigned char softuart_kbhit( unsigned char i)
{
	return( qin[i] != qout[i] );
}

void softuart_flush_input_buffer( unsigned char i )
{
	qin[i]  = 0;
	qout[i] = 0;
}
	
unsigned char softuart_transmit_busy( unsigned char i) 
{
	return ( flag_tx_busy[i] == SU_TRUE ) ? 1 : 0;
}

void softuart_putchar( unsigned char i, const char ch )
{
	while ( flag_tx_busy[i] == SU_TRUE ) {
		; // wait for transmitter ready
		  // add watchdog-reset here if needed;
	}

	// invoke_UART_transmit
	timer_tx_ctr[i]       = 3;
	bits_left_in_tx[i]    = TX_NUM_OF_BITS;
	internal_tx_buffer[i] = ( ch << 1 ) | 0x200;
	flag_tx_busy[i]       = SU_TRUE;
}
	
void softuart_puts( unsigned char i, const char *s )
{
	while ( *s ) {
		softuart_putchar( i, *s++ );
	}
}
	
void softuart_puts_p( unsigned char i, const char *prg_s )
{
	char c;

	while ( ( c = pgm_read_byte( prg_s++ ) ) ) {
		softuart_putchar(i, c);
	}
}



