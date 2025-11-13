#include "lab02.h"

#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

#define FCY_EXT 32768
volatile unsigned int ms = 0;
volatile unsigned int sec = 0;
volatile unsigned int min = 0;

void initialize_timer()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    T1CONbits.TON = 0;
    T2CONbits.TON = 0;
    T3CONbits.TON = 0;
    
    // Set Prescaler
    T1CONbits.TCKPS = 0b11;
    T2CONbits.TCKPS = 0b11; //50 kHz
    T3CONbits.TCKPS = 0b00;

    // Set Clock Source
    T1CONbits.TCS = 1;
    T2CONbits.TCS = 0;
    T3CONbits.TCS = 0;
    
    // Set Gated Timer Mode -> don't use gating
    T2CONbits.TGATE = 0;
    T1CONbits.TGATE = 0;
    T3CONbits.TGATE = 0;

    
    // T1: Set External Clock Input Synchronization -> no sync
    T1CONbits.TSYNC = 0;

    // Load Timer Periods
    PR1 = 127; //128000; //128 000 Hz -> one cycle every 0.0000078125s
    PR2 = 99; //50 000 Hz -> one cycle every 0.00002s
    PR3 = 65535; //12.8 Mhz -> one cycle every 0.000000078125s
    
    // Reset Timer Values
    TMR1 = 0;
    TMR2 = 0;

    // Set Interrupt Priority
    IPC0bits.T1IP = 1;
    IPC1bits.T2IP = 1;

    // Clear Interrupt Flags
    IFS0bits.T1IF = 0;
    IFS0bits.T2IF = 0;

    // Enable Interrupts
    IEC0bits.T1IE = 1;
    IEC0bits.T2IE = 1;

    // Enable the Timers
    T1CONbits.TON = 1;
    T2CONbits.TON = 1;
    T3CONbits.TON = 1;

}

void timer_loop()
{
    // print assignment information
    lcd_printf("Lab02: Int & Timer");
    lcd_locate(0, 1);
    lcd_printf("Group: 8");
    
    int i = 0;
    static double T3frequency = 12800000.0;
    double t2000 = 0.0;
    unsigned int timer3;
    
    while(TRUE)
    {
        i++;
        
        if (i>= 2000){
            
            i = 0; //reset cycle count value
            timer3 = TMR3; //store timer 3 value
            TOGGLELED(LED3_PORT); // toggle LED3
            
            lcd_locate(0,6);
            lcd_printf("%02u:%02u:%03u",min,sec,ms);
                        
            t2000 = (double) timer3 / T3frequency * 1000; // time in ms
            
            lcd_locate(0, 7);
            lcd_printf("c=%5u, d=%5.4fms", timer3, t2000);
            
            TMR3 = 0;           
        }
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T1Interrupt(void)
{ // invoked every 1 second
    TOGGLELED(LED2_PORT);
    IFS0bits.T1IF = 0;
    sec++;
    if(sec>=60){
        sec = 0;
        min++;
    }
}

void __attribute__((__interrupt__, __shadow__, __auto_psv__)) _T2Interrupt(void)
{ // invoked every 2 millisecond
    
    TOGGLELED(LED1_PORT);
    IFS0bits.T2IF = 0;
    ms+=2;
    if(ms>=1000){
        ms-=1000;
    }
}