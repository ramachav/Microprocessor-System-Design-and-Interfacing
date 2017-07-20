/*
************************************************************************
 ECE 362 - Mini-Project C Source File - Spring 2017
***********************************************************************
	 	   			 		  			 		  		
 Team ID: < 11 >

 Project Name: < The Mini Hooper >

 Team Members:

   - Team/Doc Leader: < Athmaram Swaminathan >      Signature: Athmaram Swaminathan
   
   - Software Leader: < Vaibhav Ramachandran >      Signature: Vaibhav Ramachandran

   - Interface Leader: < Rochak Chandra >     Signature: Rochak Chandra

   - Peripheral Leader: < Milind Shyam >    Signature: Milind Shyam


 Academic Honesty Statement:  In signing above, we hereby certify that we 
 are the individuals who created this HC(S)12 source file and that we have
 not copied the work of any other student (past or present) while completing 
 it. We understand that if we fail to honor this agreement, we will receive 
 a grade of ZERO and be subject to possible disciplinary action.

************************************************************************

 The objective of this Mini-Project is to control 2 servos using the 
 PWM outputs from a 9s12c32 microcontroller in order to operate a 
 catapult that would throw a ball at a movable basketball hoop. The SPI
 module will be used to display messages on the LCD screen every second.
 The timer module (TIM) will be used to drive the ATD sample and PWM 
 update rate, which is set to every 100ms. The RTI module is used to 
 sample the digital inputs from the 2 pushbuttons used to control the 
 servos at a rate of every 2.048ms. The ATD module is used to sample the 
 input from Channel 0 which is basically an analog D.C. voltage signal 
 provided by a potentiometer. The 2
 servos are connected to the catapult in different ways. The basketball 
 hoop is attached to a base with 4 wheels controlled using an Arduino 
 board and motor shield. The Arduino is interfaced to a bluetooth 
 module so that the Arduino can be controlled via bluetooth through an
 Android application. We also developed an Android app to control the 
 moving basketball hoop.


************************************************************************

 List of project-specific success criteria (functionality that will be
 demonstrated):

 1. SPI module was used in order to display messages on the LCD display that
 was interfaced to the microcontroller. Appropriate messages were displayed depending 
 on the analog signal from the potentiometer. Error message was displayed when the signal
 was above 90% of the maximum possible output and the appropriate distance and torque 
 percentage were displayed otherwise. Depending on whether the start pushbutton was
 pressed the Welcome Message would be displayed on the LCD screen before the catapult
 system starts up.

 2. ATD module was used to perform an ATD conversion of the 5v DC analog signal coming 
 from channel 0 which was connected to a potentiometer. ATD module was also set up to
 provide a digital input on channels 1,2,6 and 7 for the 4 pushbuttons that were used to
 control the entire set up. The pushbutton on channel 1 would control the servo which 
 controls the release of the catapult arm. The pushbutton on channel 2 controls the 
 start of the entire setup. Only once it is pushed will the circuit be functional.

 3. TIM module was used to generate a 10ms interrupt which was used to drive the 
 ATD sample and PWM output update rate. The PWM outputs would be updated every 10ms 
 and the ATD inputs would be sampled every 10ms. It was also used to increment a counter
 which would reset the servo controlling the release of the catapult once 10 seconds
 had passed by. 

 4. RTI module was used to sample the inputs from the pushbuttons every 2.048ms.
 Every 2.048ms it would generate an interrupt that would sample the inputs from
 the pushbuttons which would accordingly direct the next step the code should
 carry out.

 5. The code was stored in the on-chip flash memory and when powered up would begin
 to function completely when the start pushbutton is pressed. Thus it operates in a 
 turn-key fashion.

***********************************************************************

  Date code started: < 4/21/2017 >

  Update history (add an entry every time a significant change is made):

  Date: < 4/21/2017 >  Name: < Vaibhav Ramachandran >   Update: < Microcontroller initializations. Peripheral initializations. Setting up ATD channel 0 to sample an analog input. Setting the RTI and TIM interrupt frequencies. Control a servo's duty cycle and rotation using PWM channel 3, which outputs the ATD converted value sampled from ATD0. The output of a potentiometer is connected to ATD0. >

  Date: < 4/22/2017 >  Name: < Vaibhav Ramachandran >   Update: < Interfaced an LCD display to the 9s12c32 microcontroller via SPI module. SPI module initializations. Created several functions to aid LCD display functionality. Display messages via rdisp function when appropriate conditions are satisfied. Calculate distace travelled by the ball when released depending on torque in rdisp function. >

  Date: < 4/23/2017 >  Name: < Vaibhav Ramachandran >   Update: < Interfaced a second servo to PWM channel 1 to control the release time of the ball. Set up ATD channel 1 as a digital input to control the duty cycle and rotation of the servo motor. Interfaced a pushbutton to ATD channel 1 to provide digital input. Also added a counter to the TIM module to reset the servo back to original position after 10 sec. >
  
  Date: < 4/24/2017 >  Name: < Vaibhav Ramachandran >   Update: < Type of message displayed was changed. Torque percentage and expected range are now displayed instead of a bar showing percentage. >

  Date: < 4/25/2017 >  Name: < Vaibhav Ramachandran >   Update: < Added a start pushbutton to start up the apparatus when it is reset. Was interfaced to ATD channel 2 as a digital input. Added a welcome message to be displayed when the start pushbutton was pressed. >
  
  Date: < 4/26/2017 >  Name: < Vaibhav Ramachandran >   Update: < Added error messages that would be triggered when the potentiometer causes the servo to rotate more than it can handle. (When analog signal from ATD0 is at 90% of 5V.) >

  Date: < 4/27/2017 >  Name: < Vaibhav Ramachandran >   Update: < Provided commenting and documentation of logic being applied in the code. >
  

***********************************************************************
*/

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <mc9s12c32.h>

/* All functions after main should be initialized here */
char inchar(void);
void outchar(char x);
void rdisp(void);		// RPM display
void bco(char x);		// SCI buffered character output
void shiftout(char);	// LCD drivers (written previously)
void lcdwait(void);   // makes LCD wait for 30 cycles
void send_byte(char); // send byte
void send_i(char);    // Send command to the LCD screen
void chgline(char);   // Change line on the LCD screen
void print_c(char);   // Print a character on the LCD screen
void pmsglcd(char[]); // print a message on the LCD screen
void delay();         // delays for about 10-15 seconds

/* Variable declarations */

char leftpb	= 0;	// left pushbutton flag
char rghtpb	= 0;	// right pushbutton flag
char servopb = 0; // servo push button flag to see if servo should rotate.
char startpb = 0; // start flag to see if user wants to start the machine
char prevpb	= 0;	// previous pushbutton state
char runstp	= 0;	// motor run/stop flag
char onesec 	= 0;	// one second flag
char tensec = 0;  // ten second flag
char tenths	= 0;	// tenth of a second flag
char tin	= 0;	// SCI transmit display buffer IN pointer
char tout	= 0;	// SCI transmit display buffer OUT pointer
int seconds = 0;
int pulscnt 	= 0;	// pulse count (read from PA every second)
int left_prev = 1;  // left pushbutton previous state
int right_prev = 1; // right pushbutton previous state
int servo_prev = 0; // servo pushbutton previous state
int start_prev = 1; // start push button previous state
int tencnt = 0;     // counter to count how many tenths of a second passed
int onecnt = 0;     // counter to count how many seconds passed by
int percent_of_max; // percentage of max torque being applied
int temp;           // temporary variable to store the ATD converted value from the potentiometer input on ATD channel 0
int i,j;            // counter variables for delay function
int range;          // variable that stores the distace that the ball will travel from the catapult based on the input from the potentiometer.

#define TSIZE 81	// transmit buffer size (80 characters)
char tbuf[TSIZE];	// SCI transmit display buffer
   	   			 		  			 		       

/* Special ASCII characters */
#define CR 0x0D		// ASCII return 
#define LF 0x0A		// ASCII new line 

/* LCD COMMUNICATION BIT MASKS (note - different than previous labs) */
#define RS 0x10		// RS pin mask (PTT[4])
#define RW 0x20		// R/W pin mask (PTT[5])
#define LCDCLK 0x40	// LCD EN/CLK pin mask (PTT[6])

/* LCD INSTRUCTION CHARACTERS */
#define LCDON 0x0F	// LCD initialization command
#define LCDCLR 0x01	// LCD clear display command
#define TWOLINE 0x38	// LCD 2-line enable command
#define CURMOV 0xFE	// LCD cursor move instruction
#define LINE1 0x80	// LCD line 1 cursor position
#define LINE2 0xC0	// LCD line 2 cursor position

	 	   		
/*	 	   		
***********************************************************************
 Initializations
***********************************************************************
*/

void  initializations(void) {

/* Set the PLL speed (bus clock = 24 MHz) */
  CLKSEL = CLKSEL & 0x80; //; disengage PLL from system
  PLLCTL = PLLCTL | 0x40; //; turn on PLL
  SYNR = 0x02;            //; set PLL multiplier
  REFDV = 0;              //; set PLL divider
  while (!(CRGFLG & 0x08)){  }
  CLKSEL = CLKSEL | 0x80; //; engage PLL

/* Disable watchdog timer (COPCTL register) */
  COPCTL = 0x40   ; //COP off; RTI and COP stopped in BDM-mode

/* Initialize asynchronous serial port (SCI) for 9600 baud, interrupts off initially */
  SCIBDH =  0x00; //set baud rate to 9600
  SCIBDL =  0x9C; //24,000,000 / 16 / 156 = 9600 (approx)  
  SCICR1 =  0x00; //$9C = 156
  SCICR2 =  0x0C; //initialize SCI for program-driven operation
  DDRB   =  0x10; //set PB4 for output mode
  PORTB  =  0x10; //assert DTR pin on COM port

/* Initialize peripherals */

/*
 Initializing the PWM unit to produce a signal with the following
 characteristics on PWM output channel 1 and 3:
   - sampling frequency of approximately 300 Hz
   - left-aligned, negative polarity
   - period register = $FF (yielding a duty cycle range of 0% to 100%,
     for duty cycle register values of $00 to $FF 
   - duty register = $00 (motor initially stopped)
                         
 IMPORTANT: Need to set MODRR so that PWM Ch 1 and 3 are routed to port pin PT1 and PT3 respectively
*/ 	   			 	
    PWME = 0x0A;    //Enable channel 1 and 3
    MODRR = 0x0A;   // Routing outputs to PT1 and PT3
    PWMPOL = 0;
    PWMCTL = 0x00;
    PWMPER3 = 255;  //Period of PWM output on channel 3 is $FF
    PWMPER2 = 0;
    PWMPER1 = 255;  //Period of PWM output on channel 1 is $FF
    PWMPER0 = 0;
    PWMCAE = 0;
    PWMDTY3 = 0;
    PWMDTY2 = 0;
    PWMDTY1 = 0;
    PWMDTY0 = 0;
    PWMCLK_PCLK1 = 1;    
    PWMCLK_PCLK3 = 1;
    PWMPRCLK = 0x11;
    PWMSCLB = 0x48; //48 = 72. Ends up giving a sampling frequency of approx. 300Hz (24000000/2/2/72/255 = 300.0625Hz)
    PWMSCLA = 0x48; //48 = 72. Ends up giving a sampling frequency of approx. 300Hz (24000000/2/2/72/255 = 300.0625Hz)
    
    /* 
 Initializing the ATD to sample a D.C. input voltage (range: 0 to 5V)
 on Channel 0 (connected to a 10K-ohm potentiometer). Also setting up channels 1 and 2
 to be operated as digital inputs which are sampled at 10ms rate by the TIM module. 
 The ATD should be operated in a program-driven (i.e., non-interrupt driven), normal
 flag clear mode using nominal sample time/clock prescaler values,
 8-bit, unsigned, non-FIFO mode.
*/	 	   			 		  			 		  		
    DDRT = 0x7F;
    ATDCTL2 = 0x80;
    ATDCTL3 = 0x10;
    ATDCTL4 = 0x85;
            
/* Initialize interrupts */
/*
  Initializing the RTI for a 2.048 ms interrupt rate
*/
    CRGINT = 0x80;
    RTICTL = 0x1F;
  
/*
  Initializing SPI for baud rate of 6 Mbs, MSB first
*/
    DDRM = 0xFF;
    SPICR1 = 0x50;
    SPICR2 = 0;
    SPIBR = 0x01;
        

/* Initializing digital I/O port pins */
    DDRAD = 0;
    ATDDIEN = 0xC6;

/* 
   Initializing the LCD
     - pull LCDCLK high (idle)
     - pull R/W' low (write state)
     - turn on LCD (LCDON instruction)
     - enable two-line mode (TWOLINE instruction)
     - clear LCD (LCDCLR instruction)
     - wait for 2ms so that the LCD can wake up     
*/ 
	 PTT_PTT6 = 1;
	 PTT_PTT5 = 0;
	 send_i(LCDON);
	 send_i(TWOLINE);
	 send_i(LCDCLR);
	 lcdwait();
	      
	      
}

	 		  			 		  		
/*	 		  			 		  		
***********************************************************************
Main
***********************************************************************
*/
void main(void) {
  	DisableInterrupts
	initializations(); 		  			 		  		
	EnableInterrupts;

 for(;;) {
  
/* < start of your main loop > */ 
  
/* If right pushbutton pressed, start motor and turn on right LED (left LED off) */
     if(rghtpb) {
        runstp = 1;
        rghtpb = 0;
        PTT_PTT0 = 1;
        PTT_PTT1 = 0;
     }      
      
/* If left pushbutton pressed, stop motor and turn on left LED (right LED off) */
     if(leftpb) {
        runstp = 0;
        leftpb = 0;
        PTT_PTT0 = 0;
        PTT_PTT1 = 1;
     }
// If start pushbutton is pressed, set runstp = 1, startpb = 0, turn left LED off (right LED on), display welcome message on LCD     
     if(startpb) {
      startpb = 0;
      runstp = 1;
      
      PTT_PTT0 = 1;
      PTT_PTT1 = 0;
      send_i(LCDCLR);
      chgline(LINE1);
      pmsglcd("   Welcome to  ");
      chgline(LINE2);
      pmsglcd("The Mini Hooper!");
      delay();
     }
     
/*
 Every one-tenth second (when "tenths" flag set):
   - perform ATD conversion on Ch 0 
   - copy the converted value to the PWM duty register for Ch 3
*/
    if(tenths) {
      if(runstp == 1) {
        tenths = 0;
        ATDCTL5 = 0x10;
        while(!(ATDSTAT0 & 0x80)){ }
        temp = ATDDR0H;
        
        PWMDTY3 = ATDDR0H;
        
       }
      } 
	
/* 	   			 		  			 		  		
 Every second (when "onesec" flag set):
   - check is runstp flag is set. If it is display appropriate message on the LCD screen.
   - Set onesec flag to zero
*/  
    if(onesec) {
    if(runstp == 1) {
        rdisp();
        onesec = 0;
    }
    }
// If servopb flag is set then reset the flag to zero and rotate the servo on PWM channel 1 by 90 degrees.    
    if(servopb) {
        servopb = 0;
        PWMDTY1 = 120;
        seconds = 0;
     }
// If ten seconds have passed since servo on PWM channel 1 rotated, then reset back to original position, set tensec flag to 0.
     if(tensec) {
          tensec = 0;
          PWMDTY1 = 60;
     }  

  
   } /* loop forever */
   
}   /* do not leave main */



/*
***********************************************************************                       
 RTI interrupt service routine: RTI_ISR

 Initialized for 2.048 ms interrupt rate

  Samples state of pushbuttons (PAD1 = servo pushbutton, PAD2 = start pushbutton, PAD6 = right, PAD7 = left)

  If change in state from "high" to "low" detected, set pushbutton flag
     leftpb (for PAD7 H -> L), rghtpb (for PAD6 H -> L), servopb ((for PAD1 H -> L)
     and startpb (for PAD2 H -> L).
************************************************************************
*/

interrupt 7 void RTI_ISR(void)
{
  	// clear RTI interrupt flagt 
  	CRGFLG = CRGFLG | 0x80; 
    if(PORTAD0_PTAD6 < right_prev)
  	  rghtpb = 1;
  	right_prev = PORTAD0_PTAD6;
  	
  	if(PORTAD0_PTAD7 < left_prev)
  	  leftpb = 1;
  	left_prev = PORTAD0_PTAD7;
  	  	 
  	if(PORTAD0_PTAD1 > servo_prev)
  	  servopb = 1;
  	servo_prev = PORTAD0_PTAD1;
  	
  	if(PORTAD0_PTAD2 < start_prev)
  	  startpb = 1;
  	start_prev = PORTAD0_PTAD2;

}

/*
***********************************************************************                       
  TIM interrupt service routine
  
  Initialized for 10ms interrupt rate	 
  
  Uses variable "tencnt" to track if one-tenth second has accumulated
     and sets "tenths" flag 
                         
  Uses variable "onecnt" to track if one second has accumulated and
     sets "onesec" flag	
     
  Uses variable "tensec" to track if ten seconds have accumulated and
     sets "tensec" flag	 		  		 		
***********************************************************************
*/

interrupt 15 void TIM_ISR(void)
{
  	// clear TIM CH 7 interrupt flag 
 	TFLG1 = TFLG1 | 0x80; 
  tencnt++;
 	  if(tencnt == 10) {
 	    onecnt++;
 	    tencnt = 0;
 	    tenths = 1;
 	  }
 	  if(onecnt == 10) {
 	    onecnt = 0;
 	    onesec = 1;  
 	  
 	    seconds++;
 	  
 	  }
 	  if(seconds == 10) {
 	    seconds = 0;
 	    tensec = 1;
 	  }

}

/*
***********************************************************************                       
  SCI interrupt service routine		 		  		
***********************************************************************
*/

interrupt 20 void SCI_ISR(void)
{
    if(SCISR1_TDRE) {
      if(tout == tin)
        SCICR2_SCTIE = 0;
      else {
        SCIDRL = tbuf[tout];
        tout = (tout+1)%TSIZE;
      }
    }
}

/*
***********************************************************************                              
 Distance and torque display routine - rdisp
                         
 This function calculates the percentage of maximum torque that is
 currently being applied based on the input provided from the 
 potentiometer. It checks if the percentage is greater than 90%. If it
 is, an error message is displayed since the servo cannot handle more 
 torque. Else the appropriate distance is calculated from the 
 percentage of torque applied and it is displayed. The percentage of 
 torque applied is also displayed on the LCD's second line.

***********************************************************************
*/

void rdisp()
{
    percent_of_max = (temp * 100) / 255;
    send_i(LCDCLR);
    chgline(LINE1);
    if(percent_of_max > 90) {   
    
      pmsglcd("    Error!!!    ");
      chgline(LINE2);
      pmsglcd("Too much torque!!");
    } 
    else {     
        pmsglcd("Distance = ");
        if(percent_of_max == 90) {
          print_c(0x31);
          print_c(0x38);
          print_c(0x30);
          print_c(0x63);
          print_c(0x6D);
        }
        if(percent_of_max == 0) {
          print_c(0x30);
          print_c(0x63);
          print_c(0x6D);
        } 
        else {
            range = ((percent_of_max) * 2);
            print_c((range/100) + 48);
            print_c((range/10)%10 + 48);
            print_c((range%10) + 48);
            print_c(0x63);
            print_c(0x6D);
        }
       

      chgline(LINE2);
      pmsglcd("Torque=     ");
      if((percent_of_max/10) == 9)
        pmsglcd("100%");
      else {
        if((percent_of_max/10) == 0)
          pmsglcd("0%");
        else {
          print_c(((percent_of_max)/9)+48);
          pmsglcd("0%");
        }
      }
    }
 
}

/*
***********************************************************************                              

 Delay function - delay
 Creates a delay for about 10-15 seconds so that the LCD display shows
 the welcome message for that period of time. 

*********************************************************************** 
*/

void delay() {
  int i,j,k;
  for(i=1;i<=1000;i++) {
    for(j=1;j<=2000;j++){
    for(k=1;k<=15;k++){
    }
    }
  }
}

/*
***********************************************************************
  shiftout: Transmits the character x to external shift 
            register using the SPI.  It should shift MSB first.  
            MISO = PM[4]
            SCK  = PM[5]
***********************************************************************
*/
 
void shiftout(char x)

{
 
  // test the SPTEF bit: wait if 0; else, continue
  // write data x to SPI data register
  // wait for 30 cycles for SPI data to shift out
  int i;
  while(SPISR_SPTEF == 0){ }
  SPIDR = x;
  for(i=1;i<=30;i++){ }

}

/*
***********************************************************************
  lcdwait: Delay for approx 2 ms
***********************************************************************
*/

void lcdwait()
{
    int i,j;
    for(i=1;i<=500;i++){
      for(j=1;j<=10;j++){ }
    }
}

/*
*********************************************************************** 
  send_byte: writes character x to the LCD
***********************************************************************
*/

void send_byte(char x)
{
     // shift out character
     // pulse LCD clock line low->high->low
     // wait 2 ms for LCD to process data
     shiftout(x);
     PTT_PTT6 = 0;
     PTT_PTT6 = 1;
     PTT_PTT6 = 0;
     lcdwait();
}

/*
***********************************************************************
  send_i: Sends instruction byte x to LCD  
***********************************************************************
*/

void send_i(char x)
{
        // set the register select line low (instruction data)
        // send byte
        PTT_PTT4 = 0;
        send_byte(x);
}

/*
***********************************************************************
  chgline: Move LCD cursor to position x
  NOTE: Cursor positions are encoded in the LINE1/LINE2 variables
***********************************************************************
*/

void chgline(char x)
{
    send_i(CURMOV);
    send_i(x);
}

/*
***********************************************************************
  print_c: Print (single) character x on LCD            
***********************************************************************
*/
 
void print_c(char x)
{
     PTT_PTT4 = 1;
     send_byte(x);
}

/*
***********************************************************************
  pmsglcd: print character string str[] on LCD
***********************************************************************
*/

void pmsglcd(char str[])
{
   int i;
   for(i=0;str[i]!='\0';i++)
      print_c(str[i]);
}

/*
***********************************************************************
 Character I/O Library Routines for 9S12C32 
***********************************************************************
 Name:         inchar
 Description:  inputs ASCII character from SCI serial port and returns it
 Example:      char ch1 = inchar();
***********************************************************************
*/

char inchar(void) {
  /* receives character from the terminal channel */
        while (!(SCISR1 & 0x20)); /* wait for input */
    return SCIDRL;
}

/*
***********************************************************************
 Name:         outchar    (use only for DEBUGGING purposes)
 Description:  outputs ASCII character x to SCI serial port
 Example:      outchar('x');
***********************************************************************
*/

void outchar(char x) {
  /* sends a character to the terminal channel */
    while (!(SCISR1 & 0x80));  /* wait for output buffer empty */
    SCIDRL = x;
}