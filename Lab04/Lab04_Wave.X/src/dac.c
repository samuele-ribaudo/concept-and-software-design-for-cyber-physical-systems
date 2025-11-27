#include "dac.h"

// tristate register
#define DAC_CS_TRIS TRISDbits.TRISD8
#define DAC_SDI_TRIS TRISBbits.TRISB10
#define DAC_SCK_TRIS TRISBbits.TRISB11
#define DAC_LDAC_TRIS TRISBbits.TRISB13

// port register
#define DAC_CS_PORT PORTDbits.RD8
#define DAC_SDI_PORT PORTBbits.RB10
#define DAC_SCK_PORT PORTBbits.RB11
#define DAC_LDAC_PORT PORTBbits.RB13

// analog to digital converter 1 port configuration register
#define DAC_SDI_AD1CFG AD1PCFGLbits.PCFG10
#define DAC_SCK_AD1CFG AD1PCFGLbits.PCFG11
#define DAC_LDAC_AD1CFG AD1PCFGLbits.PCFG13

// analog to digital converter 2 port configuration register
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


void dac_convert_milli_volt(uint16_t milliVolt)
{
    float vout = milliVolt / 1000.0f;

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
    
    dac_value |= (1 << 12);

    dac_send(dac_value);
}
