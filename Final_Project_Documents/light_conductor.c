/* 4-1-2015
 Musical Light Conductor

 Timer A is used to make the speaker output appropriate frequencies for the notes

 A digital potentiometer is used to control the volume of the speaker.

 SPI Communication is used to communicate with the digital potentiometer.

 Flash memory is used to store calibration data.
 LEDs indicate what to do to calibrate.

 Multiple ADC inputs are used to control beat and volume of speaker.
*/

#include "msp430g2553.h"
#include "musical_defines.h"

#define GREEN 0x40
#define RED 0x80
#define BUTTON 0x08

//Bit positions in P1 for SPI
#define SPI_CLK 0x10
#define SPI_SIMO 0x04
#define SPI_SS 0x01

// calculate the lo and hi bytes of the bit rate divisor
#define BRLO (BIT_RATE_DIVISOR &  0xFF)
#define BRHI (BIT_RATE_DIVISOR / 0x100)

 /* declarations of functions defined later */
 void init_spi(void);
 void init_wdt(void);
 void init_adc(void);

// Global variables and parameters (all volatilel to maintain for debugger)

//stores the two bytes for the adc values
volatile unsigned int temp; 		// temporary variable used to map adc value to digital potentiometer
volatile int adc_vals[2];
volatile unsigned long updates; //update counter for debugger
volatile unsigned long last_updates;	// keep track of previous updates
volatile unsigned int counter;			// counter variale for note frequency
volatile unsigned int data_send;		// variable to store data to be sent to digital potentiometer
volatile unsigned int low, med, high;	// variables to hold low, medium, and high hand position
volatile unsigned int light_range;		// range from low to high of light values
volatile double light_to_pot_factor;	// factor used to map adc values to potentiometer values
volatile unsigned int state;			// used to determine where your hand currently is
volatile unsigned int beat_count;		// keeps track of how many beats there have been
volatile char reached_low;				// boolean variable to determine if hand has reached low after going high
volatile char reached_high;				// boolean variable to determine if hand has reached high after going low
volatile int calib_state;
volatile unsigned char last_button;
volatile unsigned char calib_done = 0;
volatile unsigned int note_counter;
volatile unsigned int prev_data_send;

volatile unsigned int WDT_per_quarter_min = 300;
volatile unsigned int quarter_count;
volatile unsigned int WDT_per_quarter;

const long Jeopardy_Melody[] = {G5,R,C6,R,G5,R,C5,R,C5,R,G5,R,C6,R,G5,R,R,R,G5,R,C6,R,G5,R,C6,R,E6,R,D6,R,C6,R,B5,R,A5,R,GS5,R,G5,R,C6,R,G5,R,C5,R,C5,R,G5,R,C6,R,G5,R,C6,R,R,R,A5,R,G5,R,F5,R,E5,R,D5,R,C5,R,R,R,AS4,R,DS5,R,AS4,R,DS4,R,AS4,R,DS5,R,AS4,R,AS4,R,DS5,R,AS4,R,DS5,R,G5,R,R,R,F5,R,DS5,R,D5,R,C5,R,B4,R,AS4,R,DS5,R,AS4,R,DS4,R,AS4,R,DS5,R,AS4,R,DS5,R,R,R,C5,R,AS4,R,GS4,R,G4,R,R,R,F4,R,DS4,R,AS3,R,DS2,R};
const double Jeopardy_Durations[] = {QN,NS,QN,NS,QN,NS,EN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,DQN,NS,EN,NS,EN,NS,EN,NS,EN,NS,EN,NS,QN,NS,QN,NS,QN,NS,EN,NS,EN,NS,QN,NS,QN,NS,HN,NS,QN,NS,EN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,EN,NS,EN,NS,EN,NS,EN,NS,EN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,EN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,QN,NS,HN,NS};
const int JeopardyLen = (sizeof(Jeopardy_Melody)/4);

const long Heart_Go_On_Melody[] = {E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,E4,R,DS4,R,E4,R,R,R,FS4,R,GS4,R,FS4,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,E4,R,B3,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,E4,R,DS4,R,E4,R,R,R,FS4,R,GS4,R,FS4,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,E4,R,B3,R,R,R,E4,R,R,R,FS4,R,R,R,B3,R,B4,R,A4,R,GS4,R,FS4,R,R,R,GS4,R,A4,R,GS4,R,FS4,R,E4,R,DS4,R,E4,R,R,R,DS4,R,CS4,R,R,R,DS4,R,CS4,R,B3,R,R,R,E4,R,FS4,R,R,R,B3,R,B4,R,A4,R,GS4,R,FS4,R,R,R,GS4,R,A4,R,GS4,R,FS4,R,E4,R,DS4,R,E4,R,DS4,R,DS4,R,E4,R,FS4,R,GS4,R,FS4,R,E4,R,R,R,R,R,R,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,E4,R,DS4,R,E4,R,FS4,R,GS4,R,FS4,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,R,R,E4,R,B3,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,E4,R,DS4,R,E4,R,R,R,FS4,R,GS4,R,FS4,R,R,R,E4,R,E4,R,E4,R,E4,R,DS4,R,E4,R,R,R,FS4,R,B3,R,R,R,E4,R,R,R,FS4,R,R,R,B3,R,B4,R,A4,R,GS4,R,FS4,R,R,R,GS4,R,A4,R,GS4,R,FS4,R,E4,R,DS4,R,E4,R,R,R,DS4,R,CS4,R,R,R,DS4,R,CS4,R,B4,R,R,R,E4,R,FS4,R,R,R,B3,R,B4,R,A4,R,GS4,R,FS4,R,R,R,GS4,R,A4,R,GS4,R,FS4,R,E4,R,DS4,R,E4,R,DS4,R,DS4,R,E4,R,FS4,R,GS4,R,FS4,R,E4,R,R,R};//end after first two bars of coda
const double Heart_Go_On_Durations[] = {DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,/*REST*/QN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,WN,NS,/*PAGE2*//*REST*/WN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,/*REST*/QN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,WN,NS,/*REST*/WN,NS,HN,NS,/*REST*/HN,NS,HN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,EN,NS,DQN,NS,/*REST*/QN,NS,DQN,NS,EN,NS,HN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,DHN,NS,/*REST*/EN,NS,SN,NS,SN,NS,/*PAGE3*/HN,NS,/*REST*/HN,NS,WN,NS,HN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,EN,NS,DQN,NS,/*REST*/QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,QN,NS,HN,NS,QN,NS,HN,NS,HN,NS,DHN,NS,/*REST*/QN,NS,/*REST*/WN,NS,/*REST*/WN,NS,/*REST*/WN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,/*PAGE4*/HN,NS,QN,NS,/*REST*/QN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,/*REST*/EN,NS,EN,NS,WN,NS,/*REST*/WN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,/*REST*/QN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,WN,NS,/*REST*/WN,NS,/*DS AL CODA*/HN,NS,/*REST*/HN,NS,HN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,EN,NS,DQN,NS,/*REST*/QN,NS,DQN,NS,EN,NS,HN,NS,DQN,NS,EN,NS,QN,NS,QN,NS,/*REST*/QN,NS,QN,NS,DHN,NS,EN,NS,SN,NS,SN,NS,HN,NS,/*REST*/HN,NS,WN,NS,HN,NS,/*REST*/QN,NS,QN,NS,HN,NS,QN,NS,EN,NS,DQN,NS,/*REST*/QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,QN,NS,QN,NS,HN,NS,QN,NS,QN,NS,HN,NS,QN,NS,HN,NS,HN,NS,6,NS,/*REST*/HN,NS};
const int Heart_Go_On_Len = (sizeof(Heart_Go_On_Melody)/4);

#define low_state 0		// hand is in low state
#define high_state 1	// hand is in high state

// bitrate = 1 bit every 4 microseconds
#define BIT_RATE_DIVISOR 32

// =======ADC Initialization and Interrupt Handler========

// Define bit masks for ADC pin and channel used as P1.4
#define ADC_INPUT_BIT_MASK_DYN 0x2
#define ADC_INPUT_BIT_MASK_BEAT 0x1
#define ADC_INCH INCH_4 | INCH_1

// define the bit mask (within P1) corresponding to output TA0
#define TA0_BIT 0x20

// Declare variables in the information memory
#pragma DATA_SECTION(isCalibration,".infoD");
volatile const int isCalibration=0;

#pragma DATA_SECTION(lowCalib,".infoD");
volatile const int lowCalib;

#pragma DATA_SECTION(medCalib,".infoD");
volatile const int medCalib;

#pragma DATA_SECTION(highCalib,".infoD");
volatile const int highCalib;

// Simple Flash Memory Operations on info D segment
// Intended to be called indirectly by the application

void eraseD(){ // erase information memory segment D
	// assumes watchdog timer already disabled (which is necessary)
	FCTL2 = FWKEY+FSSEL_2+23; // SMCLK source + divisor 24 (assuming 8Mhz clock)
	FCTL3 = FWKEY; // unlock flash (except for segment A)
	FCTL1 = FWKEY+ERASE; // setup to erase
	*( (int *) 0x1000)  = 0; // dummy write to segment D word 0
	// since this code is in flash, there is no need to explicitly wait
	// for completion since the CPU is stopped while the flash controller
	// is erasing or writing
	FCTL1=FWKEY; // clear erase and write modes
	FCTL3=FWKEY+LOCK; // relock the segment
}

void writeDword(int value, int *address){
	// write a single word.
	// NOTE: call only once for a given address between erases!
	if ( (((unsigned int) address) >= 0x1000) &&
	     (((unsigned int) address) <0x1040)  ){// inside infoD?
		FCTL3 = FWKEY; // unlock flash (except for segment A)
		FCTL1 = FWKEY + WRT ; // enable simple write
		*address = value;	// actual write to memory
		FCTL1 = FWKEY ;     // clear write mode
		FCTL3 = FWKEY+LOCK; // relock the segment
	     }
}

// Application level routine to update the three words stored in infoD
void updateData(int isC, int low, int med, int high){
	eraseD(); // clear infoD
    writeDword(isC,(int *) &isCalibration);
    writeDword(low,(int *) &lowCalib);
    writeDword(med,(int *) &medCalib);
    writeDword(high,(int *) &highCalib);
}

// Sound Production System
void init_timer(){              // initialization and start of timer
	TA0CTL |= TACLR;              // reset clock
	TA0CTL = TASSEL_2+ID_0+MC_1;  // clock source = SMCLK
	                            // clock divider=1
	                            // UP mode
	                            // timer A interrupt off
	TA0CCTL0=0; // compare mode, output 0, no interrupt enabled
	TA0CCR0 = Jeopardy_Melody[0]; // in up mode TAR=0... TACCRO-1
	P1SEL|=TA0_BIT; // connect timer output to pin
	P1DIR|=TA0_BIT;	// set direction for TA0_BIT
}


/* basic adc operations */
void start_conversion(){

	if ((ADC10CTL1 & BUSY) == 0) {	// if not already converting
		ADC10CTL0 &= ~ENC;			// disable converting
		ADC10SA=(unsigned) &(adc_vals);		// put values into array
		ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion ready
		__bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
		++updates;
	}
}

// Given a value from the adc, get a value between 0 and 128 to send to digital potentiometer
int getValueForPot() {
	// set bounds
	if (adc_vals[0] > highCalib) {// if adc is higher than threshold
		adc_vals[0] = highCalib;
	} else if (adc_vals[0] < lowCalib) {	// if adc is lower than threshold
		return 0;
	}
	// calculate number between 0 and 128
	temp = adc_vals[0] - lowCalib;
	temp = (double)temp / light_to_pot_factor;
	if (temp > 128) {
		temp = 128;
	}
	return temp;
}

// Initialization of the ADC
void init_adc(){
	ADC10CTL1= INCH_1	//input channel A0 and A1
 			  +CONSEQ_1 // sequence of channels
 			  ;
 	ADC10AE0=ADC_INPUT_BIT_MASK_DYN + ADC_INPUT_BIT_MASK_BEAT; // enable A0 and A1 analog input. This is 0x3 for pins P1.0 and P1.1
 	ADC10DTC1=2;   // two blocks per transfer
 	ADC10CTL0= SREF_0	//reference voltages are Vss and Vcc
 	          +ADC10SHT_3 //64 ADC10 Clocks for sample and hold time (slowest)
 	          +ADC10ON	//turn on ADC10
			  +ADC10IE	// enable interupt
			  +MSC		// ADC10 Multiple SamplenConversion
 	          ;

}

// ===== Watchdog Timer Interrupt Handler ====

interrupt void WDT_interval_handler(){

	unsigned char b;
	b = (P1IN & BUTTON);//read button bit
	if (last_button && (b==0)) //button pressed
	{
		switch(calib_state)//decide next action based on calibration state
				{
					case 0://not calibrating

						P1OUT &= ~RED;
						P1OUT &= ~GREEN;
						calib_state++;
						P1OUT |= RED;
						break;
					case 1://calibrating low reading
						P1OUT |= RED;
						low = adc_vals[1];//reading value for low from photoresistor through adc
						calib_state++;
						P1OUT &= ~RED;
						P1OUT |= GREEN;
						break;
					case 2://calibrating medium reading
						P1OUT &= ~RED;
						P1OUT |= GREEN;
						med = adc_vals[1];//reading value for medium from photoresistor through adc
						calib_state++;
						P1OUT |= RED;
						break;
					case 3://calibrating high reading
						P1OUT |= RED;
						high = adc_vals[1];//reading value for high from photoresistor through adc
						calib_done = 1;
						calib_state=0;
						P1OUT &= ~RED;
						P1OUT &= ~GREEN;
						break;
					default:
						break;
				}

		if (calib_done == 1)
		{
			updateData(1,low,med,high);//routine to write new calibration values to flash memory
			calib_done = 0;
			light_range = high - low;	// get the light range by subtracting low from high
			light_to_pot_factor = (double)light_range / 128.0;		// get the factor by dividing by the max value the digital potentiometer can use
			counter = 0;
		}
	}

	if ((calib_state == 1) && (isCalibration != 0))
	{
		P1OUT |= RED;
	}
	last_button = b;//store the last button reading

	if (updates > last_updates){		// if there has been a conversion updates will be greater than last_updates
		P2OUT &= ~SPI_SS;				// bring select line low to signal start of SPI communication
		UCA0TXBUF=0;					// init sending current byte of adc int value
		while (!(IFG2 & UCA0TXIFG)); // wait for TX buffer ready
		data_send = getValueForPot();	// get value between 0 and 128 based on adc value

		// change data so that it only increments or decrements from previous value by 1
		if (data_send > prev_data_send) {
			data_send++;
		} else if (data_send < prev_data_send) {
			data_send--;
		}

		// if data is over 128, set to 128
		if (data_send > 128) {
			data_send = 128;
		} else if (data_send > 65300) {	// if data is < 0, set to 0
			data_send = 0;
		}
		UCA0TXBUF=data_send;			// send data
		prev_data_send = data_send;		// store previous data_send value
		while (!(IFG2 & UCA0TXIFG)); // wait for TX buffer ready
		while (UCA0STAT & UCBUSY); // wait for the tx to complete
		P2OUT |= SPI_SS;			// bring select line back high to signal end of SPI communication

		switch (state) {
			// hand is low
			case low_state:
				// if reading is in high range and low has been reached since last high
				if (adc_vals[1] > medCalib && reached_low) {
					state = high_state;		// change state to hand is high
					beat_count++;			// increment beat count
					reached_high = 0;		// reset reached high boolean
					WDT_per_quarter = quarter_count; //this value stored will be the new quarter note "multiplier"
					quarter_count = 0;		// reset quarter_count

					// init values for first beat
					if (beat_count == 1) {
						note_counter = WDT_per_quarter * Jeopardy_Durations[0];		// set frequency to first note
						WDT_per_quarter = 500;		// default value for for WDT per quarter value
					}

					// if value is less than min, set to min
					if (WDT_per_quarter < WDT_per_quarter_min) {
						WDT_per_quarter = WDT_per_quarter_min;
					}
				}
				break;
			// hand is high
			case high_state:
				// if reading is in low reage and high has been reached since last low
				if (adc_vals[1] < medCalib && reached_high) {
					state = low_state;		// change state to hand is low
					reached_low = 0;		// reset reached low boolean
				}
				break;
		}

		// if hand is low, hand has reached low state
		if (adc_vals[1] <= (lowCalib + 2)) {
			reached_low = 1;
		}
		// if hand is high, hand has reached high state
		if (adc_vals[1] >= highCalib) {
			reached_high = 1;
		}

		last_updates = updates;	// update last_updates variable
		start_conversion();		// start a new adc conversion
	}

	if (calib_state == 0 && beat_count > 0) {

		// if note_counter value is ridiculously high, set to 1000
		if (note_counter > 5000) {
			note_counter = 1000;
		}

		if (--note_counter==0){          // decrement the counter and act only if it has reached 0
			note_counter = WDT_per_quarter * Jeopardy_Durations[counter]; // reset the down counter, only want leftmost 4 bits

			if (Jeopardy_Melody[counter] == R) {
				TACCTL0 = 0; // turn off
			} else {
				TACCTL0 |= OUTMOD_4; // turn on
				TA0CCR0 = Jeopardy_Melody[counter]; // in up mode TAR=0... TACCRO-1, clear left 4 bits
			}
			counter++;	// increment for next note to be played when if statement is entered again
			// reset counter if end of song is reached
			if(counter == JeopardyLen) {
				counter = 0;
			}
		}
	}
	else {
		TACCTL0 = 0; // turn off speaker
	}

	quarter_count++;		// increment quarter_count

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
  		     WDTCNTCL +		// (bit 3) clear watchdog timer counter
  		                 	// bit 2=0 => SMCLK is the source
  		           1     	// bits 1-0 = 10=> source/8k ~1ms
 			 );
  	IE1 |= WDTIE; // enable WDT interrupt
 }

//----------------------------------------------------------------

void init_spi(){

	UCA0CTL1 = UCSWRST; // reset
	UCA0CTL0 |= UCCKPL +UCMST + UCMODE_0 + UCSYNC+ UCMSB; // synchronous (=SPI) master 3 wire SPI, clock polarity High
	UCA0CTL1 |= UCSSEL_2; //use SCLK : 4MHz (MCP4131 supports up to 10MHz write via SPI)
	// set baud rate = SMCLK, no further division
	UCA0BR0 = BRLO;
	UCA0BR1 = BRHI;

	UCA0MCTL = 0;	// no modulation

	UCA0CTL1 &= ~UCSWRST; // **Initialize USCI **

	P2DIR |= SPI_SS;					// set P1.3 to output direction (0x8)
	P2OUT |= SPI_SS;

	// Connect I/O pins to UCA0 SPI
	P1SEL =SPI_CLK+SPI_SIMO;
	P1SEL2=SPI_CLK+SPI_SIMO;

}


/*
 * The main program just initializes everything and leaves the action to
 * the interrupt handlers!
 */

void main(){

	WDTCTL = WDTPW + WDTHOLD;       // Stop watchdog timer
	BCSCTL1 = CALBC1_8MHZ;			// 8Mhz calibration for clock
  	DCOCTL  = CALDCO_8MHZ;

  	adc_vals[0] = 0;	// initialize adc_vals[0] to 0
  	adc_vals[1] = 0;	// initialize adc_vals[1] to 0
  	updates=0; 		//update counter for debugger
  	last_updates=0; // initialize last_update to 0 like updates value
  	data_send = 0;	// variable to hold the data to be sent using SPI to the digital potentiometer
  	state = high_state;				// start out assuming the hand is high
  	beat_count = 0;					// dummy variable to count how many times there is a beat
  	reached_high = 0;	// set to false
  	reached_low = 0;	// set to false
  	note_counter = 1;	// set to 1 initially so --note_counter in WDT handler won't make note counter high
  	counter = 0;		// start at beginning of song
  	WDT_per_quarter=0;	// initial WDT_per_quarter value is 0
  	prev_data_send = 0;	// previous data_send value is 0

  	init_spi();
  	init_wdt();
  	init_adc();
  	init_timer();

  	// initialize the I/O port.  Pins 0-3-6 are used
  	P1DIR |= RED;
  	P1DIR |= GREEN;		 // Set P1.0 (RED) and P1.6 (GREEN) to be output,
  						 // P1.3 (BUTTON) is input (the default)
  	P1OUT &= ~RED;
  	P1OUT &= ~GREEN;

  	P1REN |= BUTTON;       			 // enable internal 'PULL' resistor for the button
  	P1OUT |= BUTTON;

  	//initialize calibration state depending if calibration data exists in flash memory
	if (isCalibration == 1)
	{
		calib_state = 0;
		light_range = highCalib - lowCalib;	// get the light range by subtracting low from high
		light_to_pot_factor = (double)light_range / 128.0;		// get the factor by dividing by the max value the digital potentiometer can use
	}
	else
	{
		calib_state = 1;
	}

  	start_conversion();		// do a conversion so that updates > last_updates
  	TACCTL0 = 0; // turn of speaker
 	_bis_SR_register(GIE+LPM0_bits);

}

#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    //code from reading from adc_vals and sending it to AP
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

