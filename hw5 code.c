asm(" .length 10000");
asm(" .width 132");
// Oct 2013
// TimerTone0 == produces a 1 Khz tone using TimerA, Channel 0
// Toggled on and off with a button
// Using the timer in up mode with NO INTERRUPT
//
// Sound is turned on and off by directly manipulating
// the TACCTL0 register.  The half period is not dynamically
// updated (though it can be changed in the debugger by
// changing TACCR0.
// The pushbutton is not debounced in any way!

#include "msp430g2553.h"
//-----------------------
// The following definitions allow us to adjust some of the port properties
// for the example:

// define the bit mask (within P1) corresponding to output TA0
#define TA0_BIT 0x02

// define the location for the button
// specific bit for the button
#define BUTTON_PLAY 0x08
#define BUTTON_RESTART 0x04
#define BUTTON_FAST 0x10
#define BUTTON_SLOW 0x20
#define BUTTON_SWITCH 0x80

// define the location for the on board leds
#define GREEN 0x40
#define RED 0x01
//----------------------------------

volatile unsigned char last_button0, last_button1, last_button2, last_button3, last_button4; // the state of the button bit at the last interrupt
volatile unsigned char state;	// keeps track of state of player
volatile unsigned char song;	// keeps track of which song is selected to be played
volatile double factor;			// controls speed of player
// 		notes =	  	  { C,	  C#,   D,	  D#,	 E,	   F,	F#,	  G,    A,    B,	C,	  C#,	D,  E,	 F	 F#};
const int notes[16] = {1911, 1804, 1703, 1607, 1517, 1432, 1351, 1276, 1136, 1012, 956,	 902, 851, 758,	716, 676};
//						0	   1	2	   3	 4	   5	 6	  7		8	  9		10	  11   12	13	 14	  15

// lengths				eighth, quarter, dotted_quarter, half,	dotted_half, whole,  sixtyfourth
const int lengths[7] = { 17, 	34, 		51, 		  68, 		102, 	  136, 		2};

volatile unsigned int note_counter;   // down counter for watchdog interrupt handler
unsigned volatile int counter;			// current index of note in song being played
const unsigned char rest = 0b01100000, quarter_rest = 0b00010000, eighth_rest = 0b00000000;		// commonly using rests
const unsigned int radio_length = 279;	// length of radioactive song

//										 			 0,B		 	   0,A				 0,B			   1,D				 0,A
const unsigned char radioactive[radio_length] = {0b00001001, rest, 0b00001000, rest, 0b00001001, rest, 0b00011100, rest, 0b00001000, rest,
//												 	 0,B			   0,E				 0,B			   0,A				 0,B
												 0b00001001, rest, 0b00001101, rest, 0b00001001, rest, 0b00001000, rest, 0b00001001, rest,
//												 	 2,F#			   1,E				 0,B			   0,A				 0,B
												 0b00101111, rest, 0b00011101, rest, 0b00001001, rest, 0b00001000, rest, 0b00001001, rest,
//												 	 1,D			   0,A				 1,B			   0,E				 0,B
												 0b00011100, rest, 0b00001000, rest, 0b00001001, rest, 0b00001101, rest, 0b00001001, rest,
//												 	 0,A			   1,B				 0,D			   0,A				 1,B
												 0b00001000, rest, 0b00011001, rest, 0b00001100, rest, 0b00001000, rest, 0b00011001, rest,
//												 	 0,B		 	   0,A				 0,B			   1,D				 0,A
												 0b00001001, rest, 0b00001000, rest, 0b00001001, rest, 0b00011100, rest, 0b00001000, rest,
//												 	 0,B			   0,E				 0,B			   0,A				 0,B
												 0b00001001, rest, 0b00001101, rest, 0b00001001, rest, 0b00001000, rest, 0b00001001, rest,
//												 	 2,F#			   1,E				 0,B			   0,A				 0,B
												 0b00101111, rest, 0b00011101, rest, 0b00001001, rest, 0b00001000, rest, 0b00001001, rest,
//												 	 1,D			   0,A				 1,B			   0,E				 0,B
												 0b00011100, rest, 0b00001000, rest, 0b00001001, rest, 0b00001101, rest, 0b00001001, rest,
//												 	 0,A			   1,B				 0,D			   0,A				 1,B
 												 0b00001000, rest, 0b00011001, rest, 0b00001100, rest, 0b00001000, rest, 0b00011001, rest,
//												 	 0,B		 	   0,A				 0,B			   1,D				 0,A
												 0b00001001, rest, 0b00001000, rest, 0b00001001, rest, 0b00011100, rest, 0b00001000, rest,
//												 	 0,B			   0,E				 0,B			   0,A				 0,B
												 0b00001001, rest, 0b00001101, rest, 0b00001001, rest, 0b00001000, rest, 0b00001001, rest,
//												 	 2,F#			   1,E				 0,B			   0,A				 0,B
												 0b00101111, rest, 0b00011101, rest, 0b00001001, rest, 0b00001000, rest, 0b00001001, rest,
//												 	 1,D			   0,A				 1,B			   0,E				 0,B
												 0b00011100, rest, 0b00001000, rest, 0b00001001, rest, 0b00001101, rest, 0b00001001, rest,
//												 	 0,A			   1,B				 0,D			   0,A				 1,B
 												 0b00001000, rest, 0b00011001, rest, 0b00001100, rest, 0b00001000, rest, 0b00011001, rest, //150
//												 	 5,B		 5,B		 5,B		 5,B									0,F#
 												 0b01011001, 0b01011001, 0b01011001, 0b01011001, quarter_rest, eighth_rest, 0b00000110, rest,
//												 	 1,F#			   0,F#				 2,D					  0,A				1,F#
 												 0b00011111, rest, 0b00001111, rest, 0b00101100, eighth_rest, 0b00001000, rest, 0b00011111,
//												 		   0,F#		   2,E						0,C#			  0,C#				0,C#
 												 rest, 0b00001111, 0b00101101, eighth_rest, 0b00001011, rest, 0b00001011, rest, 0b00001011,
//												 	 	   0,C#				 0,C#			   0,D				 0,D			   1,C#
 												 rest, 0b00001011, rest, 0b00001011, rest, 0b00001100, rest, 0b00001100, rest, 0b00011011, // 186
//														   1,D				 1,C#			   1,B						0,F#
 												 rest, 0b00011100, rest, 0b00011011, rest, 0b00011001, eighth_rest, 0b00000110, rest,	// 195
//												 	 1,F#			   0,F#				 2,E					  0,A				0,A
 												 0b00011111, rest, 0b00001111, rest, 0b00101101, eighth_rest, 0b00001000, rest, 0b00001000,
//												 		   0,A				 3,A									0,E			0,C#
 												 rest, 0b00001000, rest, 0b00111000, quarter_rest, eighth_rest, 0b00000100, 0b00001011,
//												 	 0,B					  0,E		  0,C#		  0,B
 												 0b00001001, eighth_rest, 0b00000100, 0b00001011, 0b00001001, quarter_rest, quarter_rest,
//												 	 2,B			   0,F#				 1,F#			   0,F#				 3,E
 												 0b00101001, rest, 0b00000110, rest, 0b00011111, rest, 0b00001111, rest, 0b00111101, rest, //229
//													 0,A			   1,F#				 0,F#			   3,E				 0,A
 												 0b00001000, rest, 0b00011111, rest, 0b00001111, rest, 0b00111101, rest, 0b00001000, rest,
//												 	 0,C#			   1,C#				 2,D			   0,C#				 0,C#
 												 0b00001011, rest, 0b00011011, rest, 0b00101100, rest, 0b00001011, rest, 0b00001011, rest,
//												 	 0,D			   0,B				 1,B			   0,F#				 1,F#
 												 0b00001100, rest, 0b00001001, rest, 0b01001001, rest, 0b00001111, rest, 0b00011111, rest,
//												 	 2,E			   0,A				 0,A			   1,A				 0,A
 												 0b00101101, rest, 0b00001000, rest, 0b00001000, rest, 0b00011000, rest, 0b00001000, rest,
//												 	 4,A					  0,E				0,C#			  0,B				5,D
 												 0b01001000, eighth_rest, 0b00000100, rest, 0b00001011, rest, 0b00001001, rest, 0b01010010, rest}; // 279

const unsigned joy_length = 204;	// length of joy to the world song
// values						  		   3,10    rest	     2,9	 rest	   1,8	   rest		 4,7
const unsigned char joy[joy_length] = {0b00111010, rest, 0b00101001, rest, 0b00011000, rest, 0b01000111,	// C,B,A,G
// values					  rest		1,5		rest	  3,4	  rest	    3,2		rest	  3,0
							  rest, 0b00010101, rest, 0b00110100, rest, 0b00110010, rest, 0b00110000,  // F,E,D,C
//							  rest		1,7		rest	  4,8	  rest		1,8		rest	  4,9
							  rest, 0b00010111, rest, 0b01001000, rest, 0b00011000, rest, 0b01001001,  // G,A,A,B
//							  rest		1,9	    rest	  4,10	  rest		1,10	rest	  1,10
							  rest, 0b00011001, rest, 0b01001010, rest, 0b00011010, rest, 0b00011010,  // B,C,C,C
//							  	  1,9	  rest		1,8			1,7		rest	  2,7		  0,5
							  0b00011001, rest, 0b00011000, 0b00010111, rest, 0b00100111, 0b00000101,  // B,A,G,G,F
//							  	  1,4	  rest		1,10	rest	  1,10		  1,9	  rest		1,8
							  0b00010100, rest, 0b00011010, rest, 0b00011010, 0b00011001, rest, 0b00011000,  // E,C,C,B,A
//							  	  1,7	  rest		2,7			0,5			1,4		rest	  1,4	  rest
							  0b00010111, rest, 0b00100111, 0b00000101, 0b00010100, rest, 0b00010100, rest,  // G,G,F,E,E
//							  	  1,4	  rest	    1,4	    rest	  1,4	  rest	    0,4			0,5
							  0b00010100, rest, 0b00010100, rest, 0b00010100, rest, 0b00000100, 0b00000101,  // E,E,E,E,F
//							  rest		4,7		rest	  0,5	  rest		0,4		rest	  1,2	  rest
							  rest, 0b01000111, rest, 0b00000101, rest, 0b00000100, rest, 0b00010010, rest, // G,F,E,D
//							  	  1,2	  rest		1,2		rest	  0,2		  0,4	  rest		4,5
							  0b00010010, rest, 0b00010010, rest, 0b00000010, 0b00000100, rest, 0b01000101,	// D,D,D,E,F
//							  rest		0,4			0,2		rest	  1,4	  rest		3,10	rest
							  rest, 0b00000100, 0b00000010, rest, 0b00010100, rest, 0b00111010, rest,  // E,D,E,C
//							  rest		1,8		rest	  2,7		  0,5		  1,4	  rest		1,5
							  rest, 0b00011000, rest, 0b00100111, 0b00000101, 0b00010100, rest, 0b00010101, // A,G,F,E,F
//							  rest		3,4		rest	  3,2	  rest		5,0
							  rest, 0b00110100, rest, 0b00110010, rest, 0b01010000, rest, // E,D,C
//							  	  3,10    rest	     2,9	 rest	   1,8	   rest		 4,7
							  0b00111010, rest, 0b00101001, rest, 0b00011000, rest, 0b01000111,	// C,B,A,G
//      					  rest		1,5		rest	  3,4	  rest	    3,2		rest	  3,0
 							  rest, 0b00010101, rest, 0b00110100, rest, 0b00110010, rest, 0b00110000,  // F,E,D,C
//							  rest		1,7		rest	  4,8	  rest		1,8		rest	  4,9
							  rest, 0b00010111, rest, 0b01001000, rest, 0b00011000, rest, 0b01001001,  // G,A,A,B
//							  rest		1,9	    rest	  4,10	  rest		1,10	rest	  1,10
							  rest, 0b00011001, rest, 0b01001010, rest, 0b00011010, rest, 0b00011010,  // B,C,C,C
//							  	  1,9	  rest		1,8			1,7		rest	  2,7		  0,5
							  0b00011001, rest, 0b00011000, 0b00010111, rest, 0b00100111, 0b00000101,  // B,A,G,G,F
//							  	  1,4	  rest		1,10	rest	  1,10		  1,9	  rest		1,8
							  0b00010100, rest, 0b00011010, rest, 0b00011010, 0b00011001, rest, 0b00011000,  // E,C,C,B,A
//							  	  1,7	  rest		2,7			0,5			1,4		rest	  1,4	  rest
							  0b00010111, rest, 0b00100111, 0b00000101, 0b00010100, rest, 0b00010100, rest,  // G,G,F,E,E
//							  	  1,4	  rest	    1,4	    rest	  1,4	  rest	    0,4			0,5
							  0b00010100, rest, 0b00010100, rest, 0b00010100, rest, 0b00000100, 0b00000101,  // E,E,E,E,F
//							  rest		4,7		rest	  0,5	  rest		0,4		rest	  1,2	  rest
							  rest, 0b01000111, rest, 0b00000101, rest, 0b00000100, rest, 0b00010010, rest, // G,F,E,D
//							  	  1,2	  rest		1,2		rest	  0,2		  0,4	  rest		4,5
							  0b00010010, rest, 0b00010010, rest, 0b00000010, 0b00000100, rest, 0b01000101,	// D,D,D,E,F
//							  rest		0,4			0,2		rest	  1,4	  rest		3,10	rest
							  rest, 0b00000100, 0b00000010, rest, 0b00010100, rest, 0b00111010, rest,  // E,D,E,C
//							  rest		1,8		rest	  2,7		  0,5		  1,4	  rest		1,5
							  rest, 0b00011000, rest, 0b00100111, 0b00000101, 0b00010100, rest, 0b00010101, // A,G,F,E,F
//							  rest		3,4		rest	  3,2	  rest		5,0
							  rest, 0b00110100, rest, 0b00110010, rest, 0b01010000};  // E,D,C

void init_timer(void); // routine to setup the timer
void init_buttons(void); // routine to setup the button

// ++++++++++++++++++++++++++
void main(){
	// setup the watchdog timer as an interval timer
	WDTCTL =(WDTPW + // (bits 15-8) password
		             // bit 7=0 => watchdog timer on
		             // bit 6=0 => NMI on rising edge (not used here)
					 // bit 5=0 => RST/NMI pin does a reset (not used here)
		     WDTTMSEL + // (bit 4) select interval timer mode
		     WDTCNTCL +  // (bit 3) clear watchdog timer counter
		     0 // bit 2=0 => SMCLK is the source
		     + 1 // bits 1-0 = 01 => source/8K
	);
	IE1 |= WDTIE;		// enable the WDT interrupt (in the system interrupt register IE1)
	BCSCTL1 = CALBC1_1MHZ;    // 1Mhz calibration for clock
	DCOCTL  = CALDCO_1MHZ;

	init_timer();  // initialize timer
	init_buttons(); // initialize the button

	P1DIR |= RED + GREEN;		// set RED and GREEN to output direction
	P1OUT &= ~(RED + GREEN);	// turn off red and green light

	TACCTL0 |= OUTMOD_4; // turn on speaker

	// initialize the state variables
	note_counter = 1;
	counter = 0;
	state = 1;
	song = 0;
	factor = 1;

	_bis_SR_register(GIE+LPM0_bits);// enable general interrupts and power down CPU
}

// +++++++++++++++++++++++++++
// Sound Production System
void init_timer(){              // initialization and start of timer
	TA0CTL |= TACLR;              // reset clock
	TA0CTL = TASSEL_2+ID_0+MC_1;  // clock source = SMCLK
	                            // clock divider=1
	                            // UP mode
	                            // timer A interrupt off
	TA0CCTL0=0; // compare mode, output 0, no interrupt enabled
	TA0CCR0 = notes[0]-1; // in up mode TAR=0... TACCRO-1
	P1SEL|=TA0_BIT; // connect timer output to pin
	P1DIR|=TA0_BIT;
}

// sets up all finve buttons as inputs
void init_buttons(){
// All GPIO's are already inputs if we are coming in after a reset
	P1DIR &= ~(BUTTON_RESTART + BUTTON_PLAY + BUTTON_FAST + BUTTON_SLOW + BUTTON_SWITCH);
	P1OUT |= (BUTTON_RESTART + BUTTON_PLAY + BUTTON_FAST + BUTTON_SLOW + BUTTON_SWITCH);
	P1REN |= (BUTTON_RESTART + BUTTON_PLAY + BUTTON_FAST + BUTTON_SLOW + BUTTON_SWITCH);
}

// conductor of piece
// also checks buttons for new input
interrupt void WDT_interval_handler(){
	unsigned char b0, b1, b2, b3, b4;
	b0 = (P1IN & BUTTON_RESTART);	// read the BUTTON_RESTART bit
	b1 = (P1IN & BUTTON_PLAY);	// read the BUTTON_PLAY bit
	b2 = (P1IN & BUTTON_SLOW);	// read the BUTTON_SLOW bit
	b3 = (P1IN & BUTTON_FAST);	// read the BUTTON_FAST bit
	b4 = (P1IN & BUTTON_SWITCH);	// read the BUTTON_SWITCH bit
	if (last_button0 && (b0==0)) { // has the button bit gone from high to low
		note_counter = 1;			// restart
		counter = 0;
	}
	if (last_button1 && (b1==0)) {
		state ^= 1;					// switch state
		if (state == 0) {
			P1OUT |= RED;			// turn on RED led
		} else {
			P1OUT &= ~RED;			// turn off RED led
		}
	}
	if (last_button2 && (b2==0)) {
		factor /= 1.25;				// slow down
	}
	if (last_button3 && (b3==0)) {
		factor *= 1.25;				// speed up
	}
	if (last_button4 && (b4==0)) {
		song ^= 1;					// switch song
		note_counter = 1;
		counter = 0;
		P1OUT ^= GREEN;				// xor green led to indicate which song is currently selected to be played
	}

	// keep track of previous button states
	last_button0 = b0;
	last_button1 = b1;
	last_button2 = b2;
	last_button3 = b3;
	last_button4 = b4;

	switch (state) {

		case 0:			// play mode
			if (song == 0) {
				if (--note_counter==0){          // decrement the counter and act only if it has reached 0
					note_counter = factor * lengths[(unsigned int)(joy[counter] >> 4)]; // reset the down counter, only want leftmost 4 bits
					unsigned char note = joy[counter];
					if (note == rest || note == quarter_rest || note == eighth_rest) {
						TACCTL0 = 0; // turn off
					} else {
						note = note & 0b00001111;
						TACCTL0 |= OUTMOD_4; // turn on
						TA0CCR0 = notes[(unsigned int) note] - 1; // in up mode TAR=0... TACCRO-1, clear left 4 bits
					}
					counter++;
					if(counter == joy_length) {
						counter = 0;
						state = 1;
						P1OUT &= ~RED;	// turn off red led
					}
				}
			} else {
				if (--note_counter==0){          // decrement the counter and act only if it has reached 0
					note_counter = factor * lengths[(unsigned int)(radioactive[counter] >> 4)]; // reset the down counter, only want leftmost 4 bits
					unsigned char note = radioactive[counter];
					if (note == rest || note == quarter_rest || note == eighth_rest) {
						TACCTL0 = 0; // turn off
					} else {
						note = note & 0b00001111;
						TACCTL0 |= OUTMOD_4; // turn on
						TA0CCR0 = notes[(unsigned int) note] - 1; // in up mode TAR=0... TACCRO-1, clear left 4 bits
					}
					counter++;
					if(counter == radio_length) {
						counter = 0;
						state = 1;
						P1OUT &= ~RED;	// turn off red led
					}
				}
			}
			break;
		case 1:			// pause mode
			TACCTL0 = 0;		// counter for frequency should be off
	}
}
// DECLARE function WDT_interval_handler as handler for interrupt 10
// using a macro defined in the msp430g2553.h include file
ISR_VECTOR(WDT_interval_handler, ".int10");
