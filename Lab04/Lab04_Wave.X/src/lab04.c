#include "lab04.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "math.h"

#include "types.h"
#include "lcd.h"
#include "led.h"
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

// signal parameter
#define sine_freq 20.0f
#define sample_rate 1000.0f
#define v_min 0.0f 
#define v_max 3.3f
#define M_PI 3.14159265358979323846

/*
 * Timer code
 */

#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define  MS_TO_TICKS(p_scaler, freq) ((FCY / (p_scaler * freq))-1)

volatile unsigned int timer_counter;

void timer_initialize()
{
    T3CONbits.TON = 0;
    T3CONbits.TCKPS = TCKPS_8;
    T3CONbits.TCS = 0;
    T3CONbits.TGATE = 0;

    PR3 = 1599;
    TMR3 = 0;

    IPC2bits.T3IP = 1;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;

    T3CONbits.TON = 1;
}

void dac_send(uint16_t command)
{
    DAC_CS_PORT = 0;
    
    int i;
    for (i = 15; i >= 0; i--) {
        DAC_SDI_PORT = (command >> i) & 1;
        DAC_SCK_PORT = 1;
        Nop(); Nop();
        
        DAC_SCK_PORT = 0;
        Nop();
    }
    
    DAC_CS_PORT = 1;
    
    DAC_LDAC_PORT = 0;
    Nop(); Nop();
    DAC_LDAC_PORT = 1;
}


void dac_set_voltage(float voltage)
{
    uint16_t code = (uint16_t)((voltage / 4.096f) * 4095.0f);
    uint16_t command = 0x3000 | (code & 0x0FFF);
    
    dac_send(command);
}


void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt(void) {
    // Calculate Current Time t
    float t = (float)timer_counter / sample_rate;

    // Calculate Sine Wave Parameters
    float v_amp = (v_max - v_min) / 2.0f;
    float v_offset = v_min + v_amp;
    float omega = 2.0f * M_PI * sine_freq;

    // Calculate Instantaneous Voltage
    float v_out = (v_amp * sin(omega * t)) + v_offset;

    // Send to DAC
    dac_set_voltage(v_out);

    // Toggle LED1
    TOGGLELED(LED1_PORT);

    timer_counter++;

    IFS0bits.T3IF = 0;
}

/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab04: Wave");
    lcd_locate(0, 1);
    lcd_printf("Group: 8");
    
    while(TRUE) { }
}
