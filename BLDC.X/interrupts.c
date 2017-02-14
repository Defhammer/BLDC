
#include "interrupts.h"
#ifndef P33FXXXX_H_
#define P33FXXXX_H_
#include <p33Fxxxx.h>
#endif

void IntInit(void)
{
    // input Capture 1
    IC1CON =0x0001;     //capture mode : every edge
    IFS0bits.IC1IF =0;  //Clear interrupt flag

    //Input Capture 2
    IC2CON =0x0001; //every edge
    IFS0bits.IC2IF =0;



}