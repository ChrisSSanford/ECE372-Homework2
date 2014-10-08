#include "p24fj64ga002.h"
#include <stdio.h>

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
                    //if(PORTBbits.RB5 == 1) //when SW1 is not pressed
//                    {
//                        state = WaitForPress;
//                        PR1 = 14400; // reset the blinking rate of the active LED
//                        TMR1 = 0; // resets the TMR1 register to 0
//                    }

                    printf("State 1");
                    if SW1 is pressed 1 time (meaning that another press is not detected within .25 s of the first press) move to state 2;
                    if SW1 is pressed 2 times (meaning that another press is detected within .25 s of the first press) move to state 3;
                    if SW1 is held for 5 seconds or more move back to state 0;

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