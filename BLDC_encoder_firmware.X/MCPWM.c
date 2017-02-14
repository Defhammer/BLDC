#include "MCPWM.h"
#include <xc.h>
void IntMCPWM()
{
    P1TCONbits.PTOPS = 0;   			// Choose PWM time base 1:1 postscale
    P1TCONbits.PTCKPS = 0;  			// Choose PWM time base 1:1 prescale
    P1TCONbits.PTMOD = 2;                       // Choose Center Aligned PWM with

    P1TPER = 4000;
    //set the output PINs to PWM
    PWM1CON1bits.PEN1L = 1;
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2L = 1;
    PWM1CON1bits.PEN2H = 1;
    PWM1CON1bits.PEN3L = 1;
    PWM1CON1bits.PEN3H = 1;
    //set pairs to complement mode.
    PWM1CON1bits.PMOD1 = 0;
    PWM1CON1bits.PMOD2 = 0;
    PWM1CON1bits.PMOD3 = 0;

    P1DC1 = 100;
    P1DC2 = 100;
    P1DC3 = 100;
    P1OVDCON = 0x0000;

    P1TCONbits.PTEN =1;

}
