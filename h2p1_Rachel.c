/**************************************************************************************************/
/*
 * File: Homework 2 Part 1
 * Name: Christopher Sanford
 * Date: 10-09-14
 *
 * Description: Program that changes states based on the length of a button
 *              press and the current state
 *
 */
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

void Setup_UART1(void);

// ******************************************************************************************* //

volatile int state = 0;
volatile int prevstate=0;
//state transitions
volatile int longPress = 0;      //variable that stops while loop in DebounceDelay()/
volatile int shortPress = 0;
volatile int doublePress = 0;
volatile int fivems = 0;

volatile int buttonPress = 0;
volatile int timerFlag = 0;

// ******************************************************************************************* //

int main(void)
{
//Uncomment the line "void Setup_UART1(void);" below if you want to Setup the UART
    //to enable use of  printf() for debugging your code, if needed.
	//Make sure the heap size is set to 512 bytes (refer to lab 0 handout)

	//void Setup_UART1(void);

	// ****************************************************************************** //
// ********************************Set Up Switch One************************************ //
// Configure TRIS register bits for switch 1 input
	TRISBbits.TRISB5 = 1;

// Configure CN register bits to enable change notifications for switch input.
	CNEN2bits.CN27IE = 1;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;

// ********************************Set Up UART************************************ //
    //RPINR18 is a regsiter for selectable input mapping (see Table 10-2) for
	// for UART1. U1RX is 8 bit value used to specifiy connection to which
	// RP pin. RP9 is used for this configuration. Physical Pin 18.
	RPINR18bits.U1RXR = 9;
    
	// RPOR4 is a register for selctable ouput mapping (see Regsiter 1019) for
	// pins RP9 and RP8. The register for RP8 is assigned to 3 to connect
	// the U1TX output for UART1 (see table 10-3). Physical Pin 17.
	RPOR4bits.RP8R = 3;
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
    
// ********************************Set Up Timer************************************ //
// Setup Timer 1 to use internal clock (Fosc/2).
	T1CONbits.TCS=0;
	T1CONbits.TGATE = 0;

// Setup Timer 1's prescaler to 1:256.
	T1CONbits.TCKPS1 = 1;   //prescale of 256
        T1CONbits.TCKPS0 = 1;
    
// Set Timer 1 to be initially off.
	T1CONbits.TON=0;

// Clear Timer 1 value and reset interrupt flag
	TMR1=0;

	IFS0bits.T1IF=0;
        IEC0bits.T1IE=1;

// Set Timer 1's period value register to value for 5 ms.
	PR1=287;  //for prescale of 256 need 287

//        if(PORTBbits.RB2 == 0 )
//        {LATA = 0x0000;}

// ********************************Main Program Operation************************************ //
	printf("State 0.\n");
    while(1)
	{
		switch(state){
                
                //State 0: wait for button press
            case 0:
                    
                break;
                
                //State 1a: wait for long press
            case 1:
                TMR1=0;
                T1CONbits.TON=1;
                while (timerFlag!=1000);
                if (PORTBbits.RB5==0 && buttonPress==1 && prevstate==0){
                    buttonPress=0;
                    timerFlag=0;
                    TMR1=0;
                    T1CONbits.TON=0;
                    while(PORTBbits.RB5 ==0);
                    state=2;
                    prevstate=1;
                    printf("State 2.\n");
                }
                else if (PORTBbits.RB5==0 && buttonPress==1 && prevstate==2){
                    buttonPress = 0;
                    timerFlag = 0;
                    TMR1=0;
                    T1CONbits.TON=0;
                    while (PORTBbits.RB5==0);
                    state=0;
                    prevstate=1;
                    printf("State 2.\n");
                }
                else {
                    TMR1=0;
                    T1CONbits.TON=0;
                    printf("State %d:\n",prevstate);
                    timerFlag=0;
                    buttonPress=0;
                    state=prevstate;
                    prevstate=1;
                    
                }
                break;
                //State 2: Corresponds to State Machine State 1
            case 2:
                buttonPress=0;
                break;
                
                //State 3: If button is pressed in state 2, figure out how long/how many times, it is pressed
            case 3:
                timerFlag=0;
                TMR1=0;
                T1CONbits.TON=1;
                while(timerFlag<50);
                if (PORTBbits.RB5==1 && buttonPress == 4){
                    state=5;
                    prevstate=3;
                    printf("State 5.\n");
                }
                else{
                    while (buttonPress<2);
                    if (timerFlag < 1000 && buttonPress == 2) {
                        state=4;
                        prevstate=3;
                        printf("State 4.\n");
                    }
                    else if (timerFlag > 1000){
                            state = 0;
                            prevstate=3;
                            printf("State 0.\n");
                        }
                    else{
                        state = 3;
                        prevstate = 3;
                    }
                }
                break;

                //State 4: button has been single pressed
            case 4:
                //wait 5ms and return to state 2
                TMR1=0;
                T1CONbits.TON=1;
                while (timerFlag < 1);
                    //turn off timer and transition states
                    timerFlag=0;
                    TMR1=0;
                    T1CONbits.TON=0;
                    printf("State 2.\n");
                    state=2;
                break;
                //state 5: button has been double pressed
            case 5:
                //wait 5ms and return to state 2
                TMR1=0;
                T1CONbits.TON=1;
                while (timerFlag < 1);
                    //turn off timer and transition states
                    timerFlag=0;
                    TMR1=0;
                    T1CONbits.TON=0;
                    printf("State 4.\n");
                    state=4;
                    
                break;
                }}
	return 0;
}

void __attribute__((interrupt,auto_psv)) _T1Interrupt(void){
   //set flag to 1 if interrupt occurs
    timerFlag = timerFlag+1;
    IFS0bits.T1IF = 0;      //reset interrupt flag


}
void __attribute__((interrupt,auto_psv)) _CNInterrupt(void)
{
    printf("number of button presses %d\n",buttonPress);
    buttonPress=buttonPress+1;
    // Clear CN interrupt flag to allow another CN interrupt to occur.
    IFS1bits.CNIF = 0;
    if(state == 0){       
        state = 1;
        prevstate=0;
        printf("state 1:");
        
    }
    
    else if(state == 2){
        state = 3;
        prevstate=2;
        printf("state 3:");
    }
}