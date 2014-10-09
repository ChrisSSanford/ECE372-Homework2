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
    // Set Timer 1's period value regsiter to value for 250ms. Please note
    // T1CON's register settings below (internal Fosc/2 and 1:256 prescalar).
    //
    //    Fosc     = XTFREQ * PLLMODE
    //             = 7372800 * 4
    //             = 29491200
    //
    //    Fosc/2   = 29491200 / 2
    //             = 14745600
    //
    //    Timer 1 Freq = (Fosc/2) / Prescaler
    //                 = 14745600 / 256
    //                 = 57600
    //
    //    PR1 = 25 ms / (1 / (T1 Freq))
    //        = 25e-3 / (1 / 57600)
    //        = 25e-3 * 57600
    //        = 1440
    PR1 = 1440;

    // Clear Timer 1 interrupt flag. This allows us to detect the
    // first interupt.
    IFS0bits.T1IF = 0;

    // Enable the interrupt for Timer 1
    IEC0bits.T1IE = 1;

    // Setup Timer 1 control register (T1CON) to:
    //     TON           = 1     (start timer)
    //     TCKPS1:TCKPS2 = 11    (set timer prescaler to 1:256)
    //     TCS           = 0     (Fosc/2)
    T1CON = 0x8030;

    // Set UART1's baud rate generator register (U1BRG) to the value calculated above.
    U1BRG  = BRGVAL;

    // Set UART1's mode register to 8-bit data, no parity, 1 stop bit, enabled.
    //     UARTEN        = 1     (enable UART)
    //     PDSEL1:PDSEL0 = 00    (8-bit data, no parity)
    //     STSEL         = 0     (1 stop bit)
    U1MODE = 0x8000;

    // Set UART2's status and control register
    //     UTXISEL1:UTXISEL0 = 00    (U1TXIF set when character
    //                                written to trasmit buffer)
    //     UTXEN             = 1     (trasnmit enabled)
    //     URXISEL1:URXISEL0 = 01    (U1RXIF set when any character
    //                                is received in receive buffer)
    //     RIDLE             = 0     (Reciver is active)
    U1STA  = 0x0440; 		// Reset status register and enable TX & RX

    // Clear the UART RX interrupt flag. Althouhg we are not using a ISR for
    // the UART receive, the UART RX interrupt flag can be used to deermine if
    // we have recived a character from he UART.
    IFS0bits.U1RXIF = 0;

        printf("\n\n\Christopher Sanford\n\r");

        // Print a message requesting the user to select a LED to toggle.
	printf("Current State: ");

	// The main loop for your microcontroller should not exit (return), as
	// the program should run as long as the device is powered on.

 
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