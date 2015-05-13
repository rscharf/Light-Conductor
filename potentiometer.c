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

//Bit positions in P1 for SPI
#define SPI_CLK 0x20
#define SPI_SOMI 0x40
#define SPI_SIMO 0x80
#define SPI_SS 0x8

// calculate the lo and hi bytes of the bit rate divisor
#define BRLO (BIT_RATE_DIVISOR &  0xFF)
#define BRHI (BIT_RATE_DIVISOR / 0x100)

#define COUNTER_VAL 1911

 /* declarations of functions defined later */
 void init_spi(void);
 void init_wdt(void);
 void init_adc(void);

// Global variables and parameters (all volatilel to maintain for debugger)

//stores the two bytes for the adc values
volatile int adc_val;   // most recent result is stored in union photo
volatile unsigned long updates; //update counter for debugger
volatile unsigned long last_updates;	// keep track of previous updates
volatile unsigned int counter;
volatile unsigned int data_send;
volatile unsigned int low, med, high;
volatile unsigned int light_range;
volatile double light_to_pot_factor;
volatile unsigned int state;
volatile unsigned int beat_count;
volatile char reached_low;
volatile char reached_high;
volatile unsigned int quarter_count=0;
volatile unsigned int WDT_per_quarter=0;

#define low_state 0
#define high_state 1

// bitrate = 1 bit every 4 microseconds
#define BIT_RATE_DIVISOR 32

// =======ADC Initialization and Interrupt Handler========

// Define bit masks for ADC pin and channel used as P1.4
#define ADC_INPUT_BIT_MASK 0x10
#define ADC_INCH INCH_4

// define the bit mask (within P1) corresponding to output TA0
#define TA0_BIT 0x02

// Sound Production System
void init_timer(){              // initialization and start of timer
	TA0CTL |= TACLR;              // reset clock
	TA0CTL = TASSEL_2+ID_0+MC_1;  // clock source = SMCLK
	                            // clock divider=1
	                            // UP mode
	                            // timer A interrupt off
	TA0CCTL0=0; // compare mode, output 0, no interrupt enabled
	TA0CCR0 = counter; // in up mode TAR=0... TACCRO-1
	P1SEL|=TA0_BIT; // connect timer output to pin
	P1DIR|=TA0_BIT;
}

/* basic adc operations */
void start_conversion(){
	if ((ADC10CTL1&ADC10BUSY) == 0){ // if not already converting...
		ADC10CTL0 |= ADC10SC;
 		ADC10SA=(unsigned) &(adc_val);
 		++updates;
 	}
}

// Given a value from the adc, get a value between 0 and 128 to send to digital potentiometer
int getValueForPot() {
	// set bounds
	if (adc_val > high) {// if adc is higher than threshold
		adc_val = high;
	} else if (adc_val < low) {	// if adc is lower than threshold
		return 0;
	}
	// calculate number between 0 and 128
	adc_val = adc_val - low;
	adc_val = (double)adc_val / light_to_pot_factor;
	return adc_val;
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
	if (updates > last_updates){		// if there has been a conversion updates will be greater than last_updates
		/*P1OUT &= ~SPI_SS;				// bring select line low to signal start of SPI communication
		UCB0TXBUF=0;					// init sending current byte of adc int value
		while (!(IFG2 & UCB0TXIFG)); // wait for TX buffer ready
		data_send = getValueForPot();	// get value between 0 and 128 based on adc value
		UCB0TXBUF=data_send;			// send data
		while (!(IFG2 & UCB0TXIFG)); // wait for TX buffer ready
		while (UCB0STAT & UCBUSY); // wait for the tx to complete
		P1OUT |= SPI_SS;*/			// bring select line back high to signal end of SPI communication

		switch (state) {
			case low_state:
				if (adc_val > med && reached_low) {
					state = high_state;
					beat_count++;
					WDT_per_quarter = quarter_count; //this value stored will be the new quarter note "multiplier"
					quarter_count = 0;
					reached_high = 0;
				}
				break;
			case high_state:
				if (adc_val < med && reached_high) {
					state = low_state;
					reached_low = 0;
				}
				break;
		}

		if (adc_val <= low) {
			reached_low = 1;
		}
		if (adc_val >= high) {
			reached_high = 1;
		}

		last_updates = updates;	// update last_updates variable
		start_conversion();		// start a new adc conversion
	}

	if (--counter==0){          // decrement the counter and act only if it has reached 0
		counter = COUNTER_VAL;
		TACCTL0 |= OUTMOD_4;
	}
	quarter_count++;
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
  		                	// bits 1-0 = 00=> source/32K
 			 );
  	IE1 |= WDTIE; // enable WDT interrupt
 }

//----------------------------------------------------------------

void init_spi(){

	UCB0CTL1 = UCSWRST; // reset
	UCB0CTL0 |= UCCKPL +UCMST + UCMODE_0 + UCSYNC+ UCMSB; // synchronous (=SPI) master 3 wire SPI, clock polarity High
	UCB0CTL1 |= UCSSEL_2; //use SCLK : 4MHz (MCP4131 supports up to 10MHz write via SPI)
	// set baud rate = SMCLK, no further division
	UCB0BR0 = BRLO;
	UCB0BR1 = BRHI;

	UCB0CTL1 &= ~UCSWRST; // **Initialize USCI **

	P1DIR |= SPI_SS;					// set P1.3 to output direction (0x8)
	P1OUT |= SPI_SS;

	// Connect I/O pins to UCB0 SPI
	P1SEL =SPI_CLK+SPI_SOMI+SPI_SIMO;
	P1SEL2=SPI_CLK+SPI_SOMI+SPI_SIMO;

}


/*
 * The main program just initializes everything and leaves the action to
 * the interrupt handlers!
 */

void main(){

	WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer
	BCSCTL1 = CALBC1_8MHZ;			// 8Mhz calibration for clock
  	DCOCTL  = CALDCO_8MHZ;

  	adc_val = 0;   // most recent result is stored in photo.whole_int
  	updates=0; 		//update counter for debugger
  	last_updates=0; // initialize last_update to 0 like updates value
  	data_send = 0;
  	counter = COUNTER_VAL;
  	low = 29;
  	med = 46;
  	high = 53;
  	light_range = high - low;
  	light_to_pot_factor = (double)light_range / 128.0;
  	state = high_state;
  	beat_count = 0;
  	reached_high = 0;	// set to false
  	reached_low = 0;	// set to false

  	init_spi();
  	init_wdt();
  	init_adc();
  	init_timer();

  	start_conversion();		// do a conversion so that updates > last_updates
  	TACCTL0 |= OUTMOD_4; // turn on speaker
 	_bis_SR_register(GIE+LPM0_bits);

}
