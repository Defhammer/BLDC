/*
 * File:   main.c
 * Author: Michael
 *
 * Created on January 15, 2017, 6:44 PM
 */


#include <xc.h>
#include "33FJ64MC802setup.h"
#include <stdio.h>
#include "MCPWM.h"
#include "UART1.h"
#include "QEI1.h"

//the number of ppr times 2 4096*2 is 8192
//the number of ppr times 2 2048*2 is 4096
// since the encoder starts at zero make sure to go minus 1 for ex. 8192-1 =8181
#define MAX_COUNT 8191

const unsigned int commutate[] ={0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000};
// this is for a 2048 PPR encoder the exact increment is 97.5 

// this is for the 4096 PPR encoder the exact increment is 195
const signed int comPos[]={
    0, 195, 390, 585, 780, 975,  
    1170,1365,1560,1755,1950,2145,  
    2340,2535,2730,2925,3120,3315,  
    3510,3705,3900,4095,4290,4485,     
    4680,4875,5070,5265,5460,5655,     
    5850,6045,6240,6435,6630,6825,     
    7020,7215,7410,7605,7800,7995,     
};

//the is the commutation sequence for the CCW direction
const int comSeq[]={
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



int main(void) {
    InitClk();
    InitIO();
    InitIC();
   InitUART1();
 
    IntMCPWM(2000);
    P1TPER = 1200;

    P1DC1 = 200;
    P1DC2 = 200;
    P1DC3 = 200;
    P1OVDCON = commutate[0];
    printf("Encoder count: %d\r\n", (int)POSCNT);
    printf("commutate count%d\r\n", i);
   
    int PosCnt =0;
    while(1)
    {
        
        if (POSCNT ==comPos[i] ){
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
            //printf("commutate count: %d\r\n", i);
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
