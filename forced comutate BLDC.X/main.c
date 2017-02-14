#if defined(__PIC24E__)
#include <p24Exxxx.h>

#elif defined (__PIC24F__)
#include <p24Fxxxx.h>

#elif defined(__PIC24H__)
#include <p24Hxxxx.h>

#elif defined(__dsPIC30F__)
#include <p30Fxxxx.h>

#elif defined (__dsPIC33E__)
#include <p33Exxxx.h>

#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>

#endif

_FOSCSEL(FNOSC_FRCPLL & IESO_OFF)
_FOSC(OSCIOFNC_ON & POSCMD_NONE &IOL1WAY_OFF& FCKSM_CSECMD)
_FWDT(FWDTEN_OFF & WINDIS_OFF)
#include "timer.h"
#include "MCPWM.h"
#include <adc.h>






const unsigned int commutate[] ={0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000};
unsigned int hallValue;
unsigned int i = 1;
unsigned int interval =4000;
const unsigned int maxComm =850;
int main(void) {
    CLKDIVbits.PLLPOST=0;
    TRISA=0xFFFF;
    AD1PCFGL =0x0000;
    TRISB =0x0000;
    InitTmr1();
    IntMCPWM();
    P1TPER = 1200;
    P1DC1 = 100;
    P1DC2 = 100;
    P1DC3 = 100;
    P1OVDCON = commutate[0];

  /*  while(1)
    {

        if(timer_expired && Counter == interval)
        {
            timer_expired =0;
            Counter = 0;
             i++;
            P1OVDCON = commutate[i];
            if (i>=6)
            i=0;
            interval--;
            if (interval <= maxComm)
            {
             interval = maxComm;
            }

        }
        

        
    }*/

    while(1)
    {
        hallValue = PORTA;
        P1OVDCON =commutate[hallValue];
    }
    return 0;
}

