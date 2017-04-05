
#include "8051.h"
#include "stdint.h"

/* This program communicates with the Pi via two gpio circuits
 * P1_6 is an input that is used to synchronise to the Pi
 * P1_7 is an output that is used to send data to the Pi
 * 
 * The purpose of the program is to return two 16bit counters
 * the actual values are delta values, i.e. the difference between the
 * current and last values. The current values are read on the fly from
 * the hardware counter timers T0 and T1 respectively, both configured to
 * operate as 16bit counters.
 *
 * The communication is initiated by the Pi setting start signal, 
 * represented by a long low period on P1_6. 
 * 
 * The program main loop waits for and detects this signal. 
 * After detecting the start signal the program reads the hardware counters
 * computes the deltas and stores the result in an output buffer.
 
 * After sending the start signal the Pi requests each bit of the 32bits
 * data at a time (two 16bit counter values) by clocking P1_6 with a high to
 * low transition. The program waits for these clock edges and shifts out 
 * the result stored in the output buffer one bit at at time
 */

/* define hardware interface to the Pi */
#define DATA_OUT P1_7
#define CLK_IN P1_6


/* define a 16bit counter to shadow the hardware timers */
union counter { 
	struct { uint8_t low;
		 uint8_t high;
	};
	uint16_t val;
};

/* define the buffer used to serialize the reply sent to the Pi
 * it contains two counters
 */
union output_buffer {
	struct {
		union counter ctr0;
		union counter ctr1;
	} counters;
	uint32_t data;
};



/* wait for the guard pulse to signal start of a transaction 
 * it must be longer than 25us
 */
void wait_guard(void) {
	uint16_t timeout;
	for (;;) {
		timeout = 0;
		while (CLK_IN == 1) {};
		do {
			timeout++;
		} while (CLK_IN == 0);
		if (timeout > 100)
			return;
	}
}

/* wait for a clock high to low transition
 * if it takes longer than 5us it is an error
 */
void wait_clk(void) {
	do {} while (CLK_IN == 0);
	do {} while (CLK_IN == 1);
}




/* compute the delta from the last counter value
 * since the hardware counter is read on the fly
 * handle any wrap around in software
 */
uint16_t refresh_counter(uint16_t *last, uint16_t curr) {

	uint16_t res;

	if (curr == *last)
		res = 0;
	else if (curr > *last)
			res = curr - *last;
		else
			res = (0xffff - *last) + curr;		
	*last = curr;
	return res;
}

/* read the hardware counter 0 on the fly
 * the hardware cascades two 8 bit counters which have to be read one at a time
 * The high byte is read first, then the low byte
 * To detect any overflow from low to high that might have occurred
 * the high byte is read a second time and if the high byte has changed
 * the sequence is repeated
 */
uint16_t counter0(void) {
	union counter tmp;
	uint8_t th;

	/* read hardware counter, high byte first and repeat if there
	 * was an overflow
	 */
	do {
		tmp.high = th = TH0;
		tmp.low = TL0;
	} while (th != TH0);

	return tmp.val;
}

/* read the hardware counter 0 on the fly
 * the hardware cascades two 8 bit counters which have to be read one at a time
 * The high byte is read first, then the low byte
 * To detect any overflow from low to high that might have occurred
 * the high byte is read a second time and if the high byte has changed
 * the sequence is repeated
 */
unsigned int counter1(void) {
	union counter tmp;
	uint8_t th;

	/* read hardware counter, high byte first and repeat if there
	 * was an overflow
	 */
	do {
		tmp.high = th = TH1;
		tmp.low = TL1;
	} while (th != TH1);

	return tmp.val;
}


/* The main program
 */
void main (void)
{
	uint8_t i;
	uint16_t counter0_last, counter1_last;
	uint32_t shiftreg;
	union output_buffer output;

	
	TMOD = 0x55;
	
	counter0_last = 0;
	counter1_last = 0;

	TR0 = 1;
	TR1 = 1;
	P1_6 = 1;



	for (;;){
		/* sync to start pulse from master */
		wait_guard();
		output.counters.ctr0.val=0;
		output.counters.ctr1.val=0;

		/* read and compute the current counter values */
		output.counters.ctr0.val = refresh_counter(&counter0_last, counter0());
		output.counters.ctr1.val = refresh_counter(&counter1_last, counter1());
		
		/* shift out the result */
		shiftreg = output.data;
		for (i=0;i<32; i++) {
			wait_clk();
			if (shiftreg & 0x80000000)
				DATA_OUT = 1;
			else
				DATA_OUT = 0;
			shiftreg = shiftreg << 1;
		}
	}	
}


