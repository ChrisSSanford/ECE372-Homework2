// ******************************************************************************************* //
//
// File:         HW2.c
// Date:         10-09-2014
// Authors:      Christopher Sanford
//
// Description:  Software code for Homework 1 for ECE 372 Fall 2014. Sample code toggles
//               a user specified LED on the Microchip 16-bit 28-pin starter board. The user
//               specified LED is received using the PIC's UART.
//
// Requirements: This softwrae code requires the MPLAB C30 Compiler or MPLAB XC16 (MPLAB C Compiler
//               for PIC24 MCUs)
//
// Credits:      Software code based upon sample code provided with Microchip 16-bit 28-pin
//               Development board.
//
// ******************************************************************************************* //

// Include file for PIC24FJ64GA002 microcontroller. This include file defines
// MACROS for special function registers (SFR) and control bits within those
// registers.

#include "p24fj64ga002.h"
#include <stdio.h>

// ******************************************************************************************* //
// Configuration bits for CONFIG1 settings.
//
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.
//
// These settings are appropriate for debugging the PIC microcontroller. If you need to
// program the PIC for standalone operation, change the COE_ON option to COE_OFF.

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF &
          BKBUG_ON & COE_ON & ICS_PGx1 &
          FWDTEN_OFF & WINDIS_OFF & FWPSA_PR128 & WDTPS_PS32768 )

// ******************************************************************************************* //
// Configuration bits for CONFIG2 settings.
// Make sure "Configuration Bits set in code." option is checked in MPLAB.
// This option can be set by selecting "Configuration Bits..." under the Configure
// menu in MPLAB.

_CONFIG2( IESO_OFF & SOSCSEL_SOSC & WUTSEL_LEG & FNOSC_PRIPLL & FCKSM_CSDCMD & OSCIOFNC_OFF &
          IOL1WAY_OFF & I2C1SEL_PRI & POSCMOD_XT )

// ******************************************************************************************* //
// Defines to simply UART's baud rate generator (BRG) regiser
// given the osicllator freqeuncy and PLLMODE.

#define XTFREQ          7372800         	  // On-board Crystal frequency
#define PLLMODE         4               	  // On-chip PLL setting (Fosc)
#define FCY             (XTFREQ*PLLMODE)/2    // Instruction Cycle Frequency (Fosc/2)

#define BAUDRATE         115200
#define BRGVAL          ((FCY/BAUDRATE)/16)-1

// ******************************************************************************************* //

typedef enum stateTypeEnum{WaitForPress, WaitForRelease, LEDToggle} stateType; //create a struct of type stateType to be used in switch statement

int main(void)
{
     stateType state;  //creates a state variable to be used in the switch statement

    while(1)
    {
        switch(state){
                case WaitForPress:
                    if(PORTBbits.RB5 == 0) // when SW1 is pressed
                    {
                        state = 0;
                        TMR1 = 0; // resets the TMR1 register to 0, EXTRA CREDIT: if this is not done then I noticed a delay while pressing SW1 before the LED changed blinking speed
                    }
                    break;

                case 0:
                    printf ("State 0");
                    T1CON.TON = 1;
                    if(TMR1 == 5 && PORTBbits.RB5 == 0)
                    state = 1;
                    else
                        state = 0;
                    //state = WaitForRelease;
                    break;

                case 1:
                    if(PORTBbits.RB5 == 1) //when SW1 is not pressed
                    {
                        state = WaitForPress;
                        PR1 = 14400; // reset the blinking rate of the active LED
                        TMR1 = 0; // resets the TMR1 register to 0
                    }

                    printf("State 1");
                    if(PORTBbits.RB5 == 0 && time=.25 seconds)
                    {
                        state = 2;

                    }
//                    if SW1 is pressed 1 time (meaning that another press is not detected within .25 s of the first press) move to state 2;
//                    if SW1 is pressed 2 times (meaning that another press is detected within .25 s of the first press) move to state 3;
//                    if SW1 is held for 5 seconds or more move back to state 0;

                    break;

                case 2:
                    printf("State 2");
                    start timer;
                    if in this state for 5ms move back to state 1;
                    break;

                case 3:
                    printf("State 3");
                    start timer;
                    if in this state for 5ms move back to state 2;
                    break;

                default:
                    break;

            //end added from power point

            }
    }
}