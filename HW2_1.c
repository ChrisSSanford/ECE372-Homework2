// ******************************************************************************************* //
//
// File:         HW2_1.c
// Title:        Homework 2 - Part 1
// Date:         10-09-2014
// Authors:      Christopher Sanford
// Student ID:   02839551
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


volatile int count = 0;

//typedef enum stateTypeEnum{WaitForPress, WaitForRelease, LEDToggle} stateType; //create a struct of type stateType to be used in switch statement

volatile int state;  //creates a state variable to be used in the switch statement

int main(void)
{

     // **TODO** SW1 of the 16-bit 28-pin Starter Board is connected to pin RB??   5.
     // Assign the TRISB bit for this pin to configure this port as an input.
       TRISBbits.TRISB5 = 1; //sets SW1 as input


       CNEN2bits.CN27IE = 1; // enables the change notification interrupt associated with SW1 (RB5)

       IFS1bits.CNIF = 0;  // sets the change notification interrupt flag to 0
       IEC1bits.CNIE = 1; //  enables the change notification interrupt


               // Varaible for character recived by UART.

	// RPINR18 is a regsiter for selectable input mapping (see Table 10-2) for
	// for UART1. U1RX is 8 bit value used to specifiy connection to which
	// RP pin. RP9 is used for this configuration. Physical Pin 18.
	RPINR18bits.U1RXR = 9;

	// RPOR4 is a register for selctable ouput mapping (see Regsiter 1019) for
	// pins RP9 and RP8. The register for RP8 is assigned to 3 to connect
	// the U1TX output for UART1 (see table 10-3). Physical Pin 17.
	RPOR4bits.RP8R = 3;

        // SW1 of the 16-bit 28-pin Starter Board is connected to pin RB5.
	// Assign the TRISB bit for this pin to configure this port as an input.
        TRISBbits.TRISB5 = 1; // Configures B5 (the button) as


        

     // Clear Timer value (i.e. current tiemr value) to 0
	TMR1 = 0;

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
	//    PR1 = 5 ms / (1 / (T1 Freq))
	//        = 5e-3 / (1 / 57600)
	//        = 5e-3 * 57600
	//        = 288
	PR1 = 288;

        // Clear Timer 1 interrupt flag. This allows us to detect the
	// first interupt.
	IFS0bits.T1IF = 0;

	// Enable the interrupt for Timer 1
	IEC0bits.T1IE = 1;

        // Setup Timer 1 control register (T1CON) to:
 	//     TON           = 0     (stop timer)
	//     TCKPS1:TCKPS2 = 11    (set timer prescaler to 1:256)
	//     TCS           = 0     (Fosc/2)
	T1CON = 0x0030;

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

	// Clear the UART RX interrupt flag. Although we are not using a ISR for
	// the UART receive, the UART RX interrupt flag can be used to deermine if
	// we have recived a character from he UART.
	IFS0bits.U1RXIF = 0;

        printf("\nState 0");

    while(1)
    {
        switch(state)
        {
            case 0:
                break;

            case 1:
                //printf("State 1\n");
                break;

            case 2:
                //printf("State 2\n");
                count = 0;
                T1CONbits.TON = 0;                      // start timer 1
                IFS0bits.T1IF = 0;
                TMR1 = 0;
                T1CONbits.TON = 1;

                if(count < 1 && PORTBbits.RB5 == 0)
                {
                    printf("\nState 3");
                    state = 3;
                }
                else if(count > 1 && PORTBbits.RB5 == 1)
                {
                    printf("\nState 4");
                    state = 4;
                }
                break;

            case 3:
                if(count >= 1)
                    state =  2;
                break;

            case 4:
                T1CONbits.TON = 1;                  // start timer 1
                while(PORTBbits.RB5 == 0);      // loop will run until SW1 is released
                T1CONbits.TON = 0;                  // stop timer 1
                TMR1 = 0;                       // reset the value in the timer 1 register

                if(count >= 1000)               // if count is greater than or equal to 1000, which means that the button was held down for 5 seconds
                {
                    state = 1;                      // move to state 1
                    printf ("\nState 1");           // print that we are in state 1
                    count = 0;                      // reset the count variable to zero
                }
                else
                {
                    state = 0; //state = WaitForRelease;
                    count = 0; // reset the count variable to zero
                }
                break;

            case 5:
                T1CONbits.TON = 1;                      // start timer 1
                while(PORTBbits.RB5 == 0);          // loop will run until SW1 is released
                T1CONbits.TON = 0;                      // stop timer 1
                TMR1 = 0;                           // reset the value in the timer 1 register

                 if(count >= 1000)                  // if count is greater than or equal to 1000, which means that the button was held down for 5 seconds
                    {
                    state = 0;                      // move to state 0
                    printf ("\nState 0");           // print that we are in state 0
                    count = 0;                      // reset count variable to zero
                    }
                 else state = 6;
                break;

            case 6:
                //printf("\nState 6");
                if(count > 50 && count < 1000 ) // if the count is greater than 50 and less than 1000 then the button was pressed for at least 0.25 seconds but less than 5 seconds
                {

                    printf ("\nState 2");       // print that we are in state 2
                    count = 0;                  // reset the count variable to zero

                    T1CONbits.TON = 0;
                    IFS0bits.T1IF = 0;
                    TMR1 = 0;
                    T1CONbits.TON = 1;
                    state = 2;                  // move to state 2
                }
                 
                break;

            case 7:
                //printf("\nState 7");
                T1CONbits.TON = 1;
                if (count == 1 && PORTBbits.RB5 == 1)
                    state = 1;
                break;
            
            default:
                break;



            }
    }



}

// ******************************************************************************************* //
// Defines an interrupt service routine that will execute whenever Timer 1's
// count reaches the specfied period value defined within the PR1 register.
//
//     _ISR and _ISRFAST are macros for specifying interrupts that
//     automatically inserts the proper interrupt into the interrupt vector
//     table
//
//     _T1Interrupt is a macro for specifying the interrupt for Timer 1
//
// The functionality defined in an interrupt should be a minimal as possible
// to ensure additional interrupts can be processed.

void _ISR _T1Interrupt(void)
   {
	IFS0bits.T1IF = 0;	// Clear Timer 1 interrupt flag to allow another Timer 1 interrupt to occur.
        count = count + 1;      // increment count variable every 5 ms

   }

void __attribute__((interrupt,auto_psv)) _CNInterrupt(void)
   {
    IFS1bits.CNIF = 0;

    if(PORTBbits.RB5 == 0 && state == 0)
        state = 4;

    else if(PORTBbits.RB5 == 0 && state == 1)
        state = 5;

    else if(PORTBbits.RB5 == 1 && state == 1)
        state = 2;

    else if(PORTBbits.RB5 == 1 && state == 1)
        state = 6;

    else if(PORTBbits.RB5 == 1 && state == 2)
        state = 7;
   }