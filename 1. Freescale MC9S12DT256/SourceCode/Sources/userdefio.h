#include "predefio.h"

UINT8 GetChar()
{
  
  // Poll for data
  do
  {
    // Nothing
  } while(SCI0SR1_RDRF == 0);
   
  // Fetch and return data from SCI0

  return SCI0DRL;
}

// Initializes SCI0 for 8N1, 9600 baud, polled I/O
// The value for the baud selection registers is determined
// using the formula:
//
// SCI0 Baud Rate = ( 2 MHz Bus Clock ) / ( 16 * SCI0BD[12:0] )
//--------------------------------------------------------------
void InitializeSerialPort(void)
{
    // Set baud rate to ~9600 (See above formula)
    SCI0BD = 13;          
    
    // 8N1 is default, so we don't have to touch SCI0CR1.
    // Enable the transmitter and receiver.
    SCI0CR2_TE = 1;
    SCI0CR2_RE = 1;
}

void InitializeLED()
{
  DDRA = 0xFF;
}


// Initializes I/O and timer settings for the demo.
//--------------------------------------------------------------       

void InitializeTimer(void)
{
  // Set the timer prescaler to %2, since the bus clock is at 2 MHz,
  // and we want the timer running at 1 MHz
  TSCR2_PR0 = 1;
  TSCR2_PR1 = 0;
  TSCR2_PR2 = 0;
    
  // Enable output compare on Channel 1
  TIOS_IOS1 = 1;
  
  // Set up output compare action to toggle Port T, bit 1
  TCTL2_OM1 = 0;
  TCTL2_OL1 = 1;
  
  // Set up timer compare value
  TC1 = TC1_VAL;
  
  // Clear the Output Compare Interrupt Flag (Channel 1) 
  TFLG1 = TFLG1_C1F_MASK;
  
  // Enable the output compare interrupt on Channel 1;
  TIE_C1I = 1;  
  
  //
  // Enable the timer
  // 
  TSCR1_TEN = 1;
   
  //
  // Enable interrupts via macro provided by hidef.h
  //
  EnableInterrupts;
 
}

// Entry point of our application code
//--------------------------------------------------------------       


//Configure signal on output port 9

void InitializePWM0() {

 PWMCLK_PCLK0=1;
 PWMPOL_PPOL0=1;
 PWMSCLA=0x50;
 PWMPER0=0xFF; 
 PWME_PWME0=1;
}

//Configure signal on output port 11
void InitializePWM1() {
 
 PWMCLK_PCLK1=1;
 PWMPOL_PPOL1=1;   
 PWMSCLA=0x50;
 PWMPER1=0xFF;
 PWME_PWME1=1;
}

// Function to call all the functions used to Initialize the System
//-----------------------------------------------------------------       
void hardwareinit(void)
{

  InitializeSerialPort();
  
  InitializeTimer();
  
  InitializePWM0();
  
  InitializePWM1();
  
  InitializeLED();
   
}