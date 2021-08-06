/*
  PololuWheelEncoders.h - Library for using Pololu Wheel Encoders.
*/
	
/*
 * Copyright (c) 2009-2012 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *   http://www.pololu.com/docs/0J18
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#ifndef PololuWheelEncoders_h
#define PololuWheelEncoders_h

#ifndef F_CPU
#define F_CPU 20000000UL
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "PololuWheelEncoders.h"
#include "OrangutanDigital.h"       // digital I/O routines
#include "OrangutanModel.h"

static char global_m1a;
static char global_m2a;
static char global_m1b;
static char global_m2b;

static int global_counts_m1;
static int global_counts_m2;

static char global_error_m1;
static char global_error_m2;

static char global_last_m1a_val;
static char global_last_m2a_val;
static char global_last_m1b_val;
static char global_last_m2b_val;

ISR(PCINT0_vect)
{
	unsigned char m1a_val = OrangutanDigital::isInputHigh(global_m1a) ? 1 : 0;
	unsigned char m2a_val = OrangutanDigital::isInputHigh(global_m2a) ? 1 : 0;
	unsigned char m1b_val = OrangutanDigital::isInputHigh(global_m1b) ? 1 : 0;
	unsigned char m2b_val = OrangutanDigital::isInputHigh(global_m2b) ? 1 : 0;

	char plus_m1 = m1a_val ^ global_last_m1b_val;
	char minus_m1 = m1b_val ^ global_last_m1a_val;
	char plus_m2 = m2a_val ^ global_last_m2b_val;
	char minus_m2 = m2b_val ^ global_last_m2a_val;

	if(plus_m1)
		global_counts_m1 += 1;
	if(minus_m1)
		global_counts_m1 -= 1;

	if(plus_m2)
		global_counts_m2 += 1;
	if(minus_m2)
		global_counts_m2 -= 1;

	if(m1a_val != global_last_m1a_val && m1b_val != global_last_m1b_val)
		global_error_m1 = 1;
	if(m2a_val != global_last_m2a_val && m2b_val != global_last_m2b_val)
		global_error_m2 = 1;

	global_last_m1a_val = m1a_val;
	global_last_m1b_val = m1b_val;
	global_last_m2a_val = m2a_val;
	global_last_m2b_val = m2b_val;
}

ISR(PCINT1_vect,ISR_ALIASOF(PCINT0_vect));
ISR(PCINT2_vect,ISR_ALIASOF(PCINT0_vect));
#ifdef PCINT3_vect
ISR(PCINT3_vect,ISR_ALIASOF(PCINT0_vect));
#endif

static void enable_interrupts_for_pin(unsigned char p)
{
	// TODO: Unify this with the code in OrangutanPulseIn::start
	// that does the same thing, and move it to OrangutanDigital.

	struct IOStruct io;
	OrangutanDigital::getIORegisters(&io, p);

#if defined(_ORANGUTAN_SVP) || defined(_ORANGUTAN_X2)
	if (io.pinRegister == &PINA)
		PCMSK0 |= io.bitmask;
	if (io.pinRegister == &PINB)
		PCMSK1 |= io.bitmask;
	if (io.pinRegister == &PINC)
		PCMSK2 |= io.bitmask;
	if (io.pinRegister == &PIND)
		PCMSK3 |= io.bitmask;
#else
	if (io.pinRegister == &PINB)
		PCMSK0 |= io.bitmask;
	if (io.pinRegister == &PINC)
		PCMSK1 |= io.bitmask;
	if (io.pinRegister == &PIND)
		PCMSK2 |= io.bitmask;
#endif

	// Preserving the old behavior of the library prior to 2012-08-21,
	// we make the line be an input but do not specify whether its pull-up
	// should be enabled or not.
	*io.ddrRegister &= ~io.bitmask;

	// For simplicity set all the bits in PCICR and let the enabling of
	// pin-change interrupts be solely controlled by PCMSKx bits.
	PCICR = 0xFF;
}


#ifdef __cplusplus

class PololuWheelEncoders
{
  public:
	/*
	 * Constructor: does nothing.
	 */
	PololuWheelEncoders() { }

	/*
	 * Initializes the wheel encoders.  The four arguments are the
	 * four pins that the two wheel encoders are connected to, according
	 * to the Arduino numbering: Arduino digital pins 0 - 7 correpsond
	 * to port D pins PD0 - PD7, respectively.  Arduino digital pins 8
	 * - 13 correspond to port B pins PB0 - PB5.  Arduino analog
	 * inputs 0 - 5 are referred to as digital pins 14 - 19 (these are
	 * the enumerations you should use for this library) and
	 * correspond to port C pins PC0 - PC5.
	 *
	 * The arguments are named m1a, m2a, etc. with the intention
	 * that when motor M1 is spinning forward, pin m1a will
	 * change before pin m1b.  However, it is difficult to get them
	 * all correct on the first try, and you might have to
	 * experiment.
	 * 
	 * init() may be called multiple times.
	 */
	static void init(unsigned char m1a, unsigned char m1b, unsigned char m2a, unsigned char m2b) {
		global_m1a = m1a;
		global_m1b = m1b;
		global_m2a = m2a;
		global_m2b = m2b;

		// disable interrupts while initializing
		cli();

		enable_interrupts_for_pin(m1a);
		enable_interrupts_for_pin(m1b);
		enable_interrupts_for_pin(m2a);
		enable_interrupts_for_pin(m2b);

		// initialize the global state
		global_counts_m1 = 0;
		global_counts_m2 = 0;
		global_error_m1 = 0;
		global_error_m2 = 0;

		global_last_m1a_val = OrangutanDigital::isInputHigh(global_m1a) ? 1 : 0;
		global_last_m1b_val = OrangutanDigital::isInputHigh(global_m1b) ? 1 : 0;
		global_last_m2a_val = OrangutanDigital::isInputHigh(global_m2a) ? 1 : 0;
		global_last_m2b_val = OrangutanDigital::isInputHigh(global_m2b) ? 1 : 0;

		// Clear the interrupt flags in case they were set before for any reason.
		// On the AVR, interrupt flags are cleared by writing a logical 1
		// to them.
		PCIFR = 0xFF;

		// enable interrupts
		sei();
	}

	/*
	 * Encoder counts are returned as integers.  For the Pololu wheel
	 * encoders, the resolution is about 3mm/count, so this allows a
	 * maximum distance of 32767*3mm or about 100m.  For longer
	 * distances, you will need to occasionally reset the counts using
	 * the functions below.
	 */
	static int getCountsM1() {
		cli();
		int tmp = global_counts_m1;
		sei();
		return tmp;
	}
	static int getCountsM2() {
		cli();
		int tmp = global_counts_m2;
		sei();
		return tmp;
	}

	/*
	 * These functions get the number of counts and reset the stored
	 * number to zero.
	 */
	static int getCountsAndResetM1();
	static int getCountsAndResetM2();

	/*
	 * These functions check whether there has been an error on M1 or
	 * M2; that is, if both m1a/m1b or m2a/m2b changed simultaneously.
	 * They return 1 if there was an error, then reset the error
	 * flag.
	 */
	static unsigned char checkErrorM1();
	static unsigned char checkErrorM2();
};

extern "C" {
#endif // __cplusplus

/*
void encoders_init(unsigned char m1a, unsigned char m1b, unsigned char m2a, unsigned char m2b);
int encoders_get_counts_m1(void);
int encoders_get_counts_m2(void);
int encoders_get_counts_and_reset_m1(void);
int encoders_get_counts_and_reset_m2(void);
int encoders_check_error_m1(void);
int encoders_check_error_m2(void);
*/

#ifdef __cplusplus
}
#endif

#endif

// Local Variables: **
// mode: C++ **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **