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

// signal parameter
#define sine_freq 10.0f
#define sample_rate 1000.0f
#define v_min 0.0f 
#define v_max 3.0f
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

    PR3 = FCY / (8 * sample_rate) - 1;
    TMR3 = 0;

    IPC2bits.T3IP = 1;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 1;

    T3CONbits.TON = 1;
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
    dac_convert_milli_volt((uint16_t)(v_out * 1000.0f));

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

