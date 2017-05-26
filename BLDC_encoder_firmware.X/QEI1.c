#include <xc.h>
#include "QEI1.h"
#include "timer.h"

//initialize the QEI module
void InitQEI(int maxCnt)
{
QEICONbits.QEIM = 0; // Disable QEI Module
QEICONbits.CNTERR = 0; // Clear any count errors
QEICONbits.QEISIDL = 0; // Continue operation during sleep
QEICONbits.SWPAB = 0; // QEA and QEB not swapped
QEICONbits.PCDOUT = 0; // Normal I/O pin operation
QEICONbits.POSRES = 1; // Index pulse resets position counter
DFLTCONbits.CEID = 1; // Count error interrupts disabled
DFLTCONbits.QEOUT = 1; // Digital filters output enabled for QEn pins
DFLTCONbits.QECK = 3; // 1:64 clock divide for digital filter for QEn
MAXCNT = maxCnt;      
POSCNT = 0; // Reset position counter
QEICONbits.QEIM = 6; // X4 mode with position counter reset by Index
return;
}

int AngPos[2] = {0,0};           // Two variables are used for Speed Calculation
int POSCNTcopy = 0;

void PositionCalculation(void)
{
    POSCNTcopy = (int)POSCNT;
    if (POSCNTcopy < 0)
        POSCNTcopy = -POSCNTcopy;
    AngPos[1] = AngPos[0];
    AngPos[0] = (unsigned int)(((unsigned long)POSCNTcopy * 4096)/125); 
            // 0 <= POSCNT <= 1999 to 0 <= AngPos <= 32752
return;
}


int Speed;
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{
IFS0bits.T1IF = 0;   // Clear timer 1 interrupt flag
PositionCalculation();
Speed = AngPos[0] - AngPos[1];
if (Speed >= 0)
{
if (Speed >= (HALFMAXSPEED))
        Speed = Speed - MAXSPEED;
}
else
{
if (Speed < -(HALFMAXSPEED))
        Speed = Speed + MAXSPEED;
}
Speed *= 2;
return;
}