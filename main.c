/* 4-1-2015
 SPI_bounce_master
 At regular WDT intervals, this sends data out the UCB SPI interface.
 The data received is logged by the RX interrupt handler.
 This example can be used to loop back (ie, connecting MOSI to MISO)

 Timing and clock.
 MCLK and SMCLK = 8 Mhz
 UCB0BRx interface divisor is a parameterized below.
 WDT divides SMCL by 512 (==> fastest rate gives 1 TX every 64 microseconds)
 TX 16 bit Parameter BIT_RATE_DIVISOR controls the SPI bitrate clock

 ADC
 * This is an example of using the ADC to convert a single
 * analog input. The external input is to ADC10 channel A4.
 * This version triggers a conversion with regular WDT interrupts and
 * uses the ADC10 interrupt to copy the converted value to a variable in memory.
*/

#include "msp430g2553.h"

 /* declarations of functions defined later */
 void init_wdt(void);
 void init_adc(void);

// Global variables and parameters (all volatilel to maintain for debugger)

//stores the two bytes for the adc values
union {
	volatile int whole_int;
	volatile char bytes[2];
} photo;   // most recent result is stored in union photo
volatile unsigned long updates; //update counter for debugger
volatile unsigned long last_updates;	// keep track of previous updates
volatile unsigned int byte_sel;			// select which byte for adc to send
volatile unsigned char data_received; 	// most recent byte receive

// bitrate = 1 bit every 4 microseconds
#define BIT_RATE_DIVISOR 32

// =======ADC Initialization and Interrupt Handler========

// Define bit masks for ADC pin and channel used as P1.4
#define ADC_INPUT_BIT_MASK 0x10
#define ADC_INCH INCH_4

/* basic adc operations */
void start_conversion(){
	if ((ADC10CTL1&ADC10BUSY) == 0){ // if not already converting...
		ADC10CTL0 |= ADC10SC;
 		ADC10SA=(unsigned) &(photo.whole_int);
 		++updates;
 	}
}

// Initialization of the ADC
void init_adc(){
	ADC10CTL1= ADC_INCH	//input channel 4
 			  +SHS_0 //use ADC10SC bit to trigger sampling
 			  +ADC10DIV_4 // ADC10 clock/5
 			  +ADC10SSEL_0 // Clock Source=ADC10OSC
 			  +CONSEQ_0; // single channel, single conversion
 			  ;
 	ADC10AE0=ADC_INPUT_BIT_MASK; // enable A4 analog input
 	ADC10DTC1=1;   // one block per transfer
 	ADC10CTL0= SREF_0	//reference voltages are Vss and Vcc
 	          +ADC10SHT_3 //64 ADC10 Clocks for sample and hold time (slowest)
 	          +ADC10ON	//turn on ADC10
 	          +ENC		//enable (but not yet start) conversions
 	          ;
}

// ===== Watchdog Timer Interrupt Handler ====

interrupt void WDT_interval_handler(){
	start_conversion();
}
ISR_VECTOR(WDT_interval_handler, ".int10")

void init_wdt(){
	// setup the watchdog timer as an interval timer
	// INTERRUPT NOT YET ENABLED!
  	WDTCTL =(WDTPW +		// (bits 15-8) password
     	                   	// bit 7=0 => watchdog timer on
       	                 	// bit 6=0 => NMI on rising edge (not used here)
                        	// bit 5=0 => RST/NMI pin does a reset (not used here)
           	 WDTTMSEL +     // (bit 4) select interval timer mode
  		     WDTCNTCL  		// (bit 3) clear watchdog timer counter
  		                	// bit 2=0 => SMCLK is the source
  		                	// bits 1-0 = 10=> source/512
 			 );
  	IE1 |= WDTIE; // enable WDT interrupt
 }

/*
 * The main program just initializes everything and leaves the action to
 * the interrupt handlers!
 */

void main(){

	WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer
	BCSCTL1 = CALBC1_8MHZ;			// 8Mhz calibration for clock
  	DCOCTL  = CALDCO_8MHZ;

  	photo.whole_int = 0;   // most recent result is stored in photo.whole_int
  	updates=0; 		//update counter for debugger
  	last_updates=0; // initialize last_update to 0 like updates value
  	byte_sel = 0;	// set first byte to be the byte to start sending
  	data_received = 0;	// initialize received data to 0

  	init_wdt();
  	init_adc();
  	start_conversion();		// do a conversion so that updates > last_updates
 	_bis_SR_register(GIE+LPM0_bits);

}
