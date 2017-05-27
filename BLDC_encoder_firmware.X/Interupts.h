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

// This is a guard condition so that contents of this file are not included
// more than once.  

int DesiredSpeed;
int ActualSpeed;
int SpeedError;
long SpeedIntegral = 0, SpeedIntegral_n_1 = 0, SpeedProportional = 0;
long DutyCycle = 0;
unsigned int Kps = 20000;					// Kp and Ks terms need to be adjusted
unsigned int Kis = 2000;					// as per the motor and load 

#include <xc.h> // include processor files - each processor file is guarded. 

void __attribute__((interrupt, no_auto_psv)) _IC1Interrupt (void)
{
  	
    unsigned int t1,t2;
    t1=IC1BUF;
    t2=IC1BUF;
    IFS0bits.IC1IF=0;
    if(t2>t1)
        timePeriod = t2-t1;
    else
        timePeriod = (PR2 - t1) + t2;
    
    if (Flags.RunMotor)  				
		DesiredSpeed = timeperiod / POTMULT;	
}

void __attribute__((interrupt, no_auto_psv)) _QEI1Interrupt (void)
{

		timer3value = TMR3;
		TMR3 = 0;
		timer3avg = ((timer3avg + timer3value) >> 1);
		polecount = 1;
		} 
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt (void)
{

	ActualSpeed = SPEEDMULT/timer3avg;
	SpeedError = DesiredSpeed - ActualSpeed;
	SpeedProportional = (int)(((long)Kps*(long)SpeedError) >> 15);
	SpeedIntegral = SpeedIntegral_n_1 + (int)(((long)Kis*(long)SpeedError) >> 15);
	
	if (SpeedIntegral < 0)
		SpeedIntegral = 0;
	else if (SpeedIntegral > 32767)
		SpeedIntegral = 32767;
	SpeedIntegral_n_1 = SpeedIntegral;
	DutyCycle = SpeedIntegral + SpeedProportional;
	if (DutyCycle < 0)
		DutyCycle = 0;
	else if (DutyCycle > 32767)
		DutyCycle = 32767;
	
	PDC1 = (int)(((long)(PTPER*2)*(long)DutyCycle) >> 15);
	PDC2 = PDC1;
	PDC3 = PDC1;
	

	IFS0bits.T1IF = 0;