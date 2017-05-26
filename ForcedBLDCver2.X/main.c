/*
 * File:   main.c
 * Author: mfkap
 *
 * Created on January 13, 2017, 8:12 PM
 */


#include "xc.h"
#include "33FJ64MC802setup.h"
#include "QEI1.h"
#include "timer.h"
#include "MCPWM.h"
#include "UART1.h"
#include <stdio.h>

    const unsigned int commutate[] ={0x0000,0x2001,0x2004,0x0204,0x0210,0x0810,0x0801,0x0000,0x0000};
    unsigned int hallValue;
    unsigned int i = 1;
    unsigned int interval =1000;
    const unsigned int maxComm =1;

void commcont(){
    while(1)
    {
       
        if(timer_expired && Counter == interval)
        {
            
            
            
            
            timer_expired =0;
            Counter = 0;
             i++;
            P1OVDCON = commutate[1];
            printf("test %d\n",i);
            if (i>=6)
            i=1;
            interval--;
            if (interval <= maxComm)
            {
             interval = maxComm;
            }

        }
   
        

        
    }
    
}    
    
    
int main(void) {
    
    initClk();
    initIO();
    InitTmr1();
    IntMCPWM(2000);
    InitUART1();
    P1DC1 = 10;
    P1DC2 = 10;
    P1DC3 = 10;
    P1OVDCON = commutate[0];
    
    while (1) {
    unsigned int mode =0;
    unsigned int commPos =0;
   
    commcont();
    
    
    }

    /*while(1)
    {
        hallValue = PORTA;
        P1OVDCON =commutate[hallValue];
    }*/
    return 0;
}


