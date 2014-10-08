int main(void)
{

    while(1)
    {
        switch(state){
                case WaitForPress:
                    if(PORTBbits.RB5 == 0) // when SW1 is pressed
                    {
                        state = 0;
                        //PR1 = 14400/2; //doubled the blinking rate of the active LED
                        TMR1 = 0; // resets the TMR1 register to 0, EXTRA CREDIT: if this is not done then I noticed a delay while pressing SW1 before the LED changed blinking speed

                    }
                    break;

                case 0:
                    //LATB = LATB^0x8000;
                    print that it is in state 0;
                    if SW1 is held for 5 seconds or more move to state 1;
                    if anything else stay in this state;
                    //state = WaitForRelease;
                    break;

                case 1:
                    //if(PORTBbits.RB5 == 1) //when SW1 is not pressed
//                    {
//                        state = WaitForPress;
//                        PR1 = 14400; // reset the blinking rate of the active LED
//                        TMR1 = 0; // resets the TMR1 register to 0
//                    }

                    print that it is in state 1;
                    if SW1 is pressed 1 time (meaning that another press is not detected within .25 s of the first press) move to state 2;
                    if SW1 is pressed 2 times (meaning that another press is detected within .25 s of the first press) move to state 3;
                    if SW1 is held for 5 seconds or more move back to state 0;

                    break;

                case 2:
                    print that it is in state 2;
                    start timer;
                    if in this state for 5ms move back to state 1;
                    break;

                case 3:
                    print that it is in state 3;
                    start timer;
                    if in this state for 5ms move back to state 2;
                    break;

                default:
                    break;

            //end added from power point

            }
    }
}