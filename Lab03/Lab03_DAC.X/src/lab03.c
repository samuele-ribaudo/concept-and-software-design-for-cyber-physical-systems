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

/*
 * Timer code
 */

#define FCY_EXT   32768UL

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

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
    
}

// interrupt service routine?

/*
 * main loop
 */

uint16_t volt_to_dac_12bit_value(float vout){
    // page 22 MCP4822-datasheet.pdf
    uint16_t dac_value = 0; // we are usign channel A (bit 15 == 0);
    float vref = 2.048; // reference voltage in volts page 4 MCP4822-datasheet.pdf
    
    if(vout > 2*vref) vout = 2*vref; // max value we can represent
    if(vout < 0) vout = 0; // vout should be positive
    
    if(vout <= vref){
        // gain = 1
        dac_value = (uit16_t) ((1 << 12) * vout/vref) | (1 << 13)
    }else{
        // gain = 2
        dac_value = (uint16_t)((1 << 12)* vout/(vref*2)) | (0 << 13)
    }
    return dac_value;
}

void main_loop()
{
    // print assignment information
    lcd_printf("Lab03: DAC");
    lcd_locate(0, 1);
    lcd_printf("Group: 8");

    uint16_t dac_value;

    float vout;
    int ms;
    int cycle_count = 0;
    
    dac_initialize();
    while(TRUE)
    {
        // main loop code
        switch(cycle_count % 3){
            case 0:
                vout = 1.0;
                ms = 500;
                cycle_count++;
            break;
            case 1:
                vout = 2.5;
                ms = 2000;
                cycle_count++;
            break;
            default;
                vout = 3.5;
                ms = 1000;
                cycle_count = 0;
        }

        dac_value = volt_to_dac_12bit_value(vout); //retrieve the register value
        
        DAC_CS_PORT = 0; // begin serial transmission;
        for(int i = 15; i >= 0; i--){
            // serially upload register value
            DAC_SDI_PORT = (dac_value >> i) & 1; // take the i bit value
            DAC_SCK_PORT = 1; // rise the clock
            Nop();
            DAC_SCK_PORT = 0;
            Nop();
        }
        DAC_CS_PORT = 1; // end transmission
        DAC_LDAC_PORT = 0; // transfer the register value from LDAC to DAC
        Nop();
        DAC_LDAC_PORT = 1;

        // delay(ms); to be implemented with timers
    }
}
