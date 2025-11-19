#include "lab03.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * DAC code
 */

#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13
    
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

#define DAC_SDI_AD2CFG AD2PCFGLbits.PCFG10
#define DAC_SCK_AD2CFG AD2PCFGLbits.PCFG11
#define DAC_LDAC_AD2CFG AD2PCFGLbits.PCFG13

void dac_initialize()
{
    // set AN10, AN11 AN13 to digital mode
    DAC_SDI_AD1CFG = 1;
    DAC_SDI_AD2CFG = 1;
    DAC_SCK_AD1CFG = 1;
    DAC_SCK_AD2CFG = 1;
    DAC_LDAC_AD1CFG = 1;
    DAC_LDAC_AD2CFG = 1;

    // this means AN10 will become RB10, AN11->RB11, AN13->RB13
    // see datasheet 11.3
    
    // set RD8, RB10, RB11, RB13 as output pins
    DAC_CS_TRIS = 0;
    DAC_SDI_TRIS = 0;
    DAC_SCK_TRIS = 0;
    DAC_LDAC_TRIS = 0;
    
    // set default state: CS=??, SCK=??, SDI=??, LDAC=??
    DAC_CS_PORT = 1; // should be set low for the duration of a write command
    DAC_SDI_PORT = 0; // don't care
    DAC_SCK_PORT = 0; // clock set on rising edge (startitng position is 0)
    DAC_LDAC_PORT = 1; // to avoid data transfer to DAC

}

void dac_send(uint16_t dac_value)
{
    DAC_CS_PORT = 0; // begin serial transmission;
    int i;
    for(i = 15; i >= 0; i--){
        // serially upload register value
        DAC_SDI_PORT = (dac_value >> i) & 1; // take the i bit value
        Nop();
        DAC_SCK_PORT = 1; // rise the clock
        Nop();
        DAC_SCK_PORT = 0;
        Nop();
    }
    DAC_CS_PORT = 1; // end transmission
    DAC_LDAC_PORT = 0; // transfer the register value from LDAC to DAC
    Nop();
    DAC_LDAC_PORT = 1;
}

void dac_set_voltage(float vout)
{
    // page 22 MCP4822-datasheet.pdf
    uint16_t dac_value = 0; // we are usign channel A (bit 15 == 0);
    float vref = 2.048; // reference voltage in volts page 4 MCP4822-datasheet.pdf
    
    if(vout > 2*vref) vout = 2*vref; // max value we can represent
    if(vout < 0) vout = 0; // vout should be positive
    
    if(vout <= vref){
        // gain = 1
        dac_value = (uint16_t) ((1 << 12) * vout/vref) | (1 << 13); // basically the formula at page 22 of MCP4822-datasheet.pdf + bit 13 value
    }else{
        // gain = 2
        dac_value = (uint16_t) ((1 << 12)* vout/(vref*2)) | (0 << 13);
    }

    dac_send(dac_value);
}

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

volatile unsigned int ms_counter = 0;

void timer_initialize()
{
    // Enable RTC Oscillator -> this effectively does OSCCONbits.LPOSCEN = 1
    // but the OSCCON register is lock protected. That means you would have to 
    // write a specific sequence of numbers to the register OSCCONL. After that 
    // the write access to OSCCONL will be enabled for one instruction cycle.
    // The function __builtin_write_OSCCONL(val) does the unlocking sequence and
    // afterwards writes the value val to that register. (OSCCONL represents the
    // lower 8 bits of the register OSCCON)
    __builtin_write_OSCCONL(OSCCONL | 2);
    // configure timer
    T1CONbits.TON = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b10;
    
    TMR1 = 0;
    PR1 = 199;
    
    IPC0bits.T1IP = 1;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    
    T1CONbits.TON = 1;
}

// interrupt service routine?
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
    ms_counter++;        // add 1 ms
    IFS0bits.T1IF = 0;   // clear interrupt flag
}

//delay function
void delay_ms(unsigned int duration)
{
    ms_counter = 0;  // reset counter
    while(ms_counter < duration); // wait
}


/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: Group 8");
    
    while(TRUE)
    {
        dac_set_voltage(1.0f);
        delay_ms(500);
        TOGGLELED(LED1_PORT);
        
        dac_set_voltage(2.5f);
        delay_ms(2000);
        TOGGLELED(LED1_PORT);
        
        dac_set_voltage(3.5f);
        delay_ms(1000);
        TOGGLELED(LED1_PORT);
    }
}
