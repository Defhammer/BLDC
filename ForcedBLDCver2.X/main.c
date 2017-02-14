/*
 * File:   main.c
 * Author: mfkap
 *
 * Created on January 13, 2017, 8:12 PM
 */


#include "xc.h"
#include "timer.h"
#include "MCPWM.h"

    const unsigned int commutate[] ={0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000,0x0000};
    unsigned int hallValue;
    unsigned int i = 1;
    unsigned int interval =6000;
    const unsigned int maxComm =850;
int main(void) {
    CLKDIVbits.PLLPOST=0;
    TRISA=0xFFFF;
    AD1PCFGL =0x0000;
    TRISB =0x0000;
    InitTmr1();
    IntMCPWM();
    P1TPER = 1200;
    P1DC1 = 1200;
    P1DC2 = 1200;
    P1DC3 = 1200;
    P1OVDCON = commutate[0];

   while(1)
    {

        if(timer_expired && Counter == interval)
        {
            timer_expired =0;
            Counter = 0;
             i++;
            P1OVDCON = commutate[i];
            if (i>=8)
            i=0;
            interval--;
            if (interval <= maxComm)
            {
             interval = maxComm;
            }

        }
        

        
    }

    /*while(1)
    {
        hallValue = PORTA;
        P1OVDCON =commutate[hallValue];
    }*/
    return 0;
}


