
void InitTmr1()
{
    /* clear T1CON settings  this defaults to: 
     timer is stopped
     continue module operation in Idle mode
     input  precaler is 1:1
     use the interal clock
     gated time accumulation is disabled */  
    
    T1CON = 0;

    IFS0bits.T1IF =0;  //clear Timer 1 interrupt flag. This is alway required
    IPC0bits.T1IP =4; //timer interupt set to prioity 4
    IEC0bits.T1IE = 1; // enable timer 1 interupt
    PR1 = 40000; // run the intterupt when the timer 1 counter reaches PERIOD
    T1CONbits.TCKPS1 = 0; //timer 1 prescaler set to 1:1
    T1CONbits.TCS =0;  //interal clock is to be used
    T1CONbits.TON =1;  // turn on timer 1

}


/*at timer 1 interupt*/
unsigned int timer_expired =0;
unsigned int Counter =0;
void __attribute__((interrupt, auto_psv)) _T1Interrupt(void)
{
    timer_expired = 1;
    LATBbits.LATB4 =~LATBbits.LATB4;
    Counter++;
    IFS0bits.T1IF =0;  //reset the timer interupt
}

void InitTMR3(void)
{
	T3CON = 0x0030;			// internal Tcy/256 clock
	TMR3 = 0;
	PR3 = 0xFFFF;
}