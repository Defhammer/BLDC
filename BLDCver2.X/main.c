/*
 * File:   main.c
 * Author: mfkap
 *
 * Created on January 13, 2017, 7:16 PM
 */


#include "xc.h"
#include "timer.h"
#include "MCPWM.h"
int main(void) {
    int unsigned commCW[] =
    {0x0000,
    0b0000001000010000,
    0b0000100000000001,
    0b0000100000010000,
    0b0010000000000100,
    0b0000001000000100,
    0b0010000000000001,
    0x0000}; 
    unsigned int hallValue;   
    CLKDIVbits.PLLPOST=1;

    TRISB =    0b0000001110000000;
    AD1PCFGL = 0b1111111111111111;

    InitTmr1();
    IntMCPWM();
    P1TPER = 10000;
    P1DC1 = 512;
    P1DC2 = 512;
    P1DC3 = 512;
    P1OVDCON = commCW[0];
    while (1)
    {
        hallValue = PORTB & 0b0000001110000000;
        hallValue = hallValue >> 7;
        P1OVDCON =commCW[hallValue];
    }   
    return 0;
}
