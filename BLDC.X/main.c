/*
 * File:   newmain.c
 * Author: Michael Kaplan
 *
 * Created on January 27, 2012, 1:09 PM
 */

/******************************************************************************
 * Software License Agreement
 *
 * Copyright � 2011 Microchip Technology Inc.  All rights reserved.
 * Microchip licenses to you the right to use, modify, copy and distribute
 * Software only when embedded on a Microchip microcontroller or digital
 * signal controller, which is integrated into your product or third party
 * product (pursuant to the sublicense terms in the accompanying license
 * agreement).
 *
 * You should refer to the license agreement accompanying this Software
 * for additional information regarding your rights and obligations.
 *
 * SOFTWARE AND DOCUMENTATION ARE PROVIDED ìAS ISî WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
 * OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
 * PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
 * OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 * BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
 * DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
 * INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
 * COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
 * CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
 * OR OTHER SIMILAR COSTS.
 *
 *****************************************************************************/


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
#include "timer.h"
#include "MCPWM.h"

_FOSCSEL(FNOSC_FRCPLL & IESO_OFF)
_FOSC(OSCIOFNC_ON & POSCMD_NONE &IOL1WAY_OFF& FCKSM_CSECMD)
_FWDT(FWDTEN_OFF & WINDIS_OFF)


//	Function Prototypes
int main(void);
int unsigned commCW[] =
{0x0000,
0b0000001000010000,
0b0000100000000001,
0b0000100000010000,
0b0010000000000100,
0b0000001000000100,
0b0010000000000001,
0x0000};

//int unsigned commCCW[] =
//{0x0000


//0x0000};
unsigned int hallValue;

int main(void)
{
    CLKDIVbits.PLLPOST=1;

    TRISB =    0b0000001110000000;
    AD1PCFGL = 0b1111111111111111;

    InitTmr1();
    IntMCPWM();
    P1TPER = 10000;
    P1DC1 = 50;
    P1DC2 = 50;
    P1DC3 = 50;
    P1OVDCON = commCW[0];
    while (1)
    {
        hallValue = PORTB & 0b0000001110000000;
        hallValue = hallValue >> 7;
        P1OVDCON =commCW[hallValue];
    }
}



