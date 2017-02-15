/*
 * File:   main.c
 * Author: Michael
 *
 * Created on January 15, 2017, 6:44 PM
 */

// DSPIC33FJ64MC802 Configuration Bit Settings

// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Mode (Internal Fast RC (FRC) w/ PLL)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF              // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD3               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "xc.h"
#include <stdio.h>
#include "MCPWM.h"
#include "timer.h"
#include "UART1.h"
#include "QEI1.h"

/*
 //OSCTUNbits.TUN=0b010011;
    AD1PCFGL=0b111111;
    TRISB =0x0000;
    TRISBbits.TRISB0=1;
    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
    
   // RPINR18bits.U1RXR=0x00;
   // RPOR0bits.RP1R = 0x03;
    //RPINR14bits.QEA1R =0x07;
   // RPINR14bits.QEB1R =0x08;
   // RPINR15bits.INDX1R=0x09;
    //InitUART1();
    //InitQEI();
  
    InitTmr1();
    IntMCPWM();
    
    P1TPER = 1200;
    P1DC1 = 100;
    P1DC2 = 100;
    P1DC3 = 100;
    P1OVDCON = commutate[0];

*/
//the number of ppr times 2 4096*2 is 8192
//the number of ppr times 2 2048*2 is 4096
// since the encoder starts at zero make sure to go minus 1 for ex. 8192-1 =8181
#define MAX_COUNT 4095

const unsigned int commutate[] ={0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000};
// this is for a 2048 PPR encoder the exact increment is 97.5 
const signed int comPos2048[]={
    0, 97, 195, 292, 390, 487,
    585 ,682 ,780 ,877 ,975 ,1072,
    1170,1267,1365,1462,1560,1657,
    1755,1852,1950,2047,2145,2242,
    2340,2437,2535,2632,2730,2827,
    2925,3022,3120,3217,3315,3412,
    3510,3607,3705,3802,3900,3997,
};
// this is for the 4096 PPR encoder the exact increment is 195
const signed int comPos4096[]={
    0, 195, 390, 585, 780, 975,  
    1170,1365,1560,1755,1950,2145,  
    2340,2535,2730,2925,3120,3315,  
    3510,3705,3900,4095,4290,4485,     
    4680,4875,5070,5265,5460,5655,     
    5850,6045,6240,6435,6630,6825,     
    7020,7215,7410,7605,7800,7995,     
};

//the is the commutation sequence for the CCW direction
int comSeq[]={
    1, 6, 5, 4, 3, 2,
    1, 6, 5, 4, 3, 2,
    1, 6, 5, 4, 3, 2,
    1, 6, 5, 4, 3, 2,
    1, 6, 5, 4, 3, 2,
    1, 6, 5, 4, 3, 2,
    1, 6, 5, 4, 3, 2,
};


unsigned int hallValue;
unsigned int i = 0;
unsigned int j = 1;
unsigned int interval =4000;
const unsigned int maxComm =850;
//printf("Encoder count: %d\r\n", (int)POSCNT); 

int main(void) {
    int comPos[] =comPos2048[];
    OSCTUNbits.TUN=0b010011;
    CLKDIVbits.PLLPRE =0;
    PLLFBDbits.PLLDIV =0x26;
    CLKDIVbits.PLLPOST =0;
    TRISA=0xFFFF;
    AD1PCFGL =0x0000;
    TRISB =0x0000;
    TRISBbits.TRISB0=1;
    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB8 = 1;
    TRISBbits.TRISB9 = 1;
      
    RPINR18bits.U1RXR=0x00;
    RPOR0bits.RP1R = 0x03;
    RPINR14bits.QEA1R =0x07;
    RPINR14bits.QEB1R =0x08;
    RPINR15bits.INDX1R=0x09;
    InitUART1();
    InitQEI(MAX_COUNT);
    InitTmr1();
    IntMCPWM();
    P1TPER = 1200;
    P1DC1 = 3300;
    P1DC2 = 3300;
    P1DC3 = 3300;
    P1OVDCON = commutate[7];
    printf("Encoder count: %d\r\n", (int)POSCNT);
    printf("commutate count%d\r\n", i);
   
    int PosCnt =0;
    while(1)
    {
        
        if (POSCNT <=comPos[i] && POSCNT >= (comPos[i]-100)){
            PosCnt = comSeq[i];
            P1OVDCON = commutate[PosCnt]; 
            i++;
        }
        
        if (i == 41){
            i=0;
        }
        
        
       
           printf("pos %d tgt %d\r\n", (int)POSCNT,comPos[i]);  
        
       
            
        /*if(timer_expired && Counter == interval)
        {
            timer_expired =0;
            Counter = 0;
             i++;
            P1OVDCON = commutate[i];
            printf("commutate count: %d\r\n", i);
            if (i>=6)
            i=0;
            interval--;
            if (interval <= maxComm)
            {
             interval = maxComm;
            }
            printf("Encoder count: %d\r\n", (int)POSCNT);
          
        }
        */
        //printf("%d\r\n", (int)POSCNT); 
    }
}
