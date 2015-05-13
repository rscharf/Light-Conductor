/*
	working flash code to save calibration data to memory

	flash_calib.c
*/
#include  "msp430g2553.h"


#define GREEN 0x40
#define RED 1
#define BUTTON 8

#define ADC_INPUT_BIT_MASK 0x10
#define ADC_INCH INCH_4

volatile int adc_val;   // most recent result is stored in union photo
volatile unsigned long updates; //update counter for debugger
volatile unsigned long last_updates;	// keep track of previous updates

volatile int calib_state;
volatile unsigned char last_button;
volatile unsigned char calib_done = 0;
volatile int low;
volatile int med;
volatile int high;

// Declare variables in the information memory
#pragma DATA_SECTION(isCalibration,".infoD");
volatile const int isCalibration=0;

#pragma DATA_SECTION(lowCalib,".infoD");
volatile const int lowCalib;

#pragma DATA_SECTION(medCalib,".infoD");
volatile const int medCalib;

#pragma DATA_SECTION(highCalib,".infoD");
volatile const int highCalib;


void start_conversion(){
	if ((ADC10CTL1&ADC10BUSY) == 0){ // if not already converting...
		ADC10CTL0 |= ADC10SC;
 		ADC10SA=(unsigned) &(adc_val);
 		++updates;
 	}
}

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

// Simple Flash Memory Operations on info D segment
// Intended to be called indirectly by the application

void eraseD(){ // erase information memory segment D
	// assumes watchdog timer already disabled (which is necessary)
	FCTL2 = FWKEY+FSSEL_2+23; // SMCLK source + divisor 24 (assuming 8Mhz clock)
	FCTL3 = FWKEY; // unlock flash (except for segment A)
	FCTL1 = FWKEY+ERASE; // setup to erase
	*( (int *) 0x1000)  = 0; // dummy write to segment D word 0
	/* since this code is in flash, there is no need to explicitly wait
	 * for completion since the CPU is stopped while the flash controller
	 * is erasing or writing
	 */
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


//-----------------------

// =============== main program
void main(void)
{

  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
  BCSCTL1 = CALBC1_8MHZ;	// usual 8Mhz clock -- this matters!!!
  DCOCTL  = CALDCO_8MHZ;

  // setup the watchdog timer as an interval timer
  WDTCTL =(WDTPW +	// (bits 15-8) password
                        // bit 7=0 => watchdog timer on
                        // bit 6=0 => NMI on rising edge (not used here)
                        // bit 5=0 => RST/NMI pin does a reset (not used here)
           WDTTMSEL +   // (bit 4) select interval timer mode
           WDTCNTCL + 	// (bit 3) clear watchdog timer counter
  		                // bit 2=0 => SMCLK is the source
  		   1            // bits 1-0 = 01 => source/8K ~ 1ms.
  		   );
  IE1 |= WDTIE;		// enable the WDT interrupt (in the system interrupt register IE1)


  // initialize the I/O port.  Pins 0-3-6 are used
  //P1DIR = RED+GREEN;                 // Set P1.0 (RED) and P1.6 (GREEN) to be output,
                                     // P1.3 (BUTTON) is input (the default)

  P1DIR |= RED;
  P1DIR |= GREEN;

  P1OUT &= ~GREEN;
  P1OUT &= ~RED;

  P1REN |= BUTTON;       			 // enable internal 'PULL' resistor for the buttomn
  P1OUT |= BUTTON;					 // LEDs off/ switch is a pullup

  adc_val = 0;   // most recent result is stored in photo.whole_int
  updates=0; 		//update counter for debugger
  last_updates=0; // initialize last_update to 0 like updates value
  init_adc();
  start_conversion();

  //initialize calibration state depending if calibration data exists in flash memory
    if (isCalibration == 1)
    {
    	calib_state = 0;
    }
    else
    {
    	calib_state = 1;
    }
    _bis_SR_register(GIE+LPM0_bits);

}


interrupt void WDT_interval_handler(){
	if (updates > last_updates)
	{
		last_updates = updates;	// update last_updates variable
		start_conversion();		// start a new adc conversion
	}


	unsigned char b;
	b = (P1IN & BUTTON);
	if (last_button && (b==0)) //button pressed
	{
		switch(calib_state)
				{
					case 0:
						P1OUT &= ~RED;
						P1OUT &= ~GREEN;
						calib_state++;
						P1OUT |= RED;
						break;
					case 1:
						P1OUT |= RED;
						//low = 65;
						low = adc_val;
						calib_state++;
						P1OUT &= ~RED;
						P1OUT |= GREEN;
						break;
					case 2:
						P1OUT &= ~RED;
						P1OUT |= GREEN;
						//med = 80;
						med = adc_val;
						calib_state++;
						P1OUT |= RED;
						break;
					case 3:
						P1OUT |= RED;
						//high = 148;
						high = adc_val;
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
			updateData(1,low,med,high);
			calib_done = 0;
		}

	}

	if ((calib_state == 1) && (isCalibration != 0))
	{
		P1OUT |= RED;
	}
	last_button = b;
}
// DECLARE WDT_interval_handler as handler for interrupt 10
ISR_VECTOR(WDT_interval_handler, ".int10")
