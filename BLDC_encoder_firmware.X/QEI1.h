/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */
#define MAX_CNT_PER_REV (2048 * 4 - 1)
#define MAXSPEED (unsigned int)(((unsigned long)MAX_CNT_PER_REV*2048)/125)
#define HALFMAXSPEED (MAXSPEED>>1)

extern int AngPos[2];           // Two variables are used for Speed Calculation
extern int POSCNTcopy;
extern int Speed;
void InitQEI(int maxCnt);
void PositionCalculation(void);


#include <xc.h> // include processor files - each processor file is guarded.  

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
DFLTCONbits.QECK = 5; // 1:64 clock divide for digital filter for QEn
//MAXCNT = maxCnt;      
POSCNT = 0; // Reset position counter
QEICONbits.QEIM = 6; // X4 mode with position counter reset by Index
return;
}

int AngPos[2] = {0,0}; // Two variables are used for Speed Calculation
int POSCNTcopy = 0;
void PositionCalculation(void)
{
POSCNTcopy = (int)POSCNT;
if (POSCNTcopy < 0)
POSCNTcopy = -POSCNTcopy;
AngPos[1] = AngPos[0];
AngPos[0] = (unsigned int)(((unsigned long)POSCNTcopy * 2048)/125);
// 0 <= POSCNT <= 1999 to 0 <= AngPos <= 32752
return;
}

void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt (void);


