/*
	flash_mem.c

	prototype
*/

#include "msp430g2553.h"

#define BUTTON	0x08
#define RED		0x01
#define GREEN	0x40


#pragma DATA_SECTION(isCalibration,".infoD");
volatile const int isCalibration = 0;

#pragma DATA_SECTION(lowCalib,".infoD");
volatile const int lowCalib;

#pragma DATA_SECTION(medCalib,".infoD");
volatile const int medCalib;

#pragma DATA_SECTION(highCalib,".infoD");
volatile const int highCalib;

volatile unsigned char calib_done = 0;
volatile unsigned char last_button;
volatile int calib_state;
volatile int low;
volatile int med;
volatile int high;

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

void updateData(int isC, int low, int med, int high)
{
	eraseD(); //clear infoD
	writeDword(isC,(int *) &isCalibration);
	writeDword(low,(int *) &low);
	writeDword(med,(int *) &med);
	writeDword(high,(int *) &high);
}

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


//initialze button
  P1REN = BUTTON;
  P1OUT = BUTTON;
  P1IES = BUTTON;
  P1IFG &= ~BUTTON;
  P1IE = BUTTON;

//initialze LEDs
  P1DIR |= RED;
  P1OUT &= ~RED;

  P1DIR |= GREEN;
  P1OUT &= ~GREEN;

//initialize calibration state depending if calibration data exists in flash memory
  if (isCalibration == 0)
  {
  	calib_state = 1;
  }
  else
  {
  	calib_state = 0;
  }
}

interrupt void WDT_interval_handler()
{
	unsigned char b;
	b = (P1IN & BUTTON);
	if (last_button && (b==0)) //button pressed
	{
		//code for on button action

		//get value from ADC
		switch(calib_state)
		{
			case 0://not calibrating
				break;
			case 1://low calibration
				//low = value from ADC
				break;
			case 2://med calibration
				//med = value from ADC
				break;
			case 3://high calibration
				//high = value from ADC
				calib_done = 1;
				calib_state = 0;
				break;
			default:
				break;
		}

		if (calib_done == 0)
		{
			calib_state++;
		}

		if (calib_done == 1)
		{
			updateData(1, low, med, high);
			calib_done = 0;
		}
	}

	switch(calib_state)
	{
		case 0://not calibrating
			P1OUT &= ~RED;
			P1OUT &= ~GREEN;
			//not calibrating, so WDT handler can carry on its merry way and do whatever it needs to do other than calibration here...
			break;
		case 1://low calibration
			P1OUT |= RED;
			break;
		case 2://med calibration
			P1OUT &= ~RED;
			P1OUT |= GREEN;
			break;
		case 3://high calibration
			P1OUT |= RED;
			break;
		default:
			break;
	}

	last_button = b;
}
ISR_VECTOR(WDT_interval_handler, ".int10")
