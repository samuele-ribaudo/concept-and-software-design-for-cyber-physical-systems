#include "lab05.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>

#include "types.h"
#include "lcd.h"
#include "led.h"

/*
 * PWM code
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

#define SERVO_X 0
#define SERVO_Y 1
#define TOUCH_X 0
#define TOUCH_Y 1
#define TMR2_PERIOD 3999

void servo_initialize(void) {

    T2CONbits.TON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = TCKPS_64;
    TMR2 = 0;
    PR2 = TMR2_PERIOD;

    TRISDbits.TRISD7 = 0;
    OC8CONbits.OCM = 0x0006;
    OC8CONbits.OCTSEL = 0;
    OC8RS = TMR2_PERIOD;

    TRISDbits.TRISD6 = 0;
    OC7CONbits.OCM = 0x0006;
    OC7CONbits.OCTSEL = 0;
    OC7RS = TMR2_PERIOD;

    T2CONbits.TON = 1;
}

void servo_set_duty(uint8_t servo, uint16_t duty_us) {
    if (duty_us < 900) duty_us = 900;
    if (duty_us > 2100) duty_us = 2100;

    // Convert microseconds to ticks (5us per tick)
    uint16_t ticks = duty_us / 5;
    
    uint16_t inverted_duty = TMR2_PERIOD - ticks;

    if (servo == SERVO_X) {
        OC8RS = inverted_duty;
    } else if (servo == SERVO_Y) {
        OC7RS = inverted_duty;
    }
}

/*
 * touch screen code
 */

void touch_initialize(void) {
    CLEARBIT(AD1CON1bits.ADON);

    CLEARBIT(TRISEbits.TRISE1); // E1
    CLEARBIT(TRISEbits.TRISE2); // E2
    CLEARBIT(TRISEbits.TRISE3); // E3

    // Set TRIS bits to Input
    SETBIT(TRISBbits.TRISB15);
    SETBIT(TRISBbits.TRISB9);

    CLEARBIT(AD1PCFGLbits.PCFG15); // AN15 Analog
    CLEARBIT(AD1PCFGLbits.PCFG9);  // AN9 Analog

    AD1CON1bits.FORM = 0;       // Integer Output
    AD1CON1bits.SSRC = 0x7;     // Auto-Conversion
    AD1CON1bits.ASAM = 0;       // Manual Sampling (we will set SAMP bit manually)
    AD1CON1bits.AD12B = 0;      // 10-bit mode
    
    AD1CON2 = 0;                // MUX A only, no scan
    
    AD1CON3bits.ADRC = 0;       // System Clock
    AD1CON3bits.SAMC = 0x1F;    // Sample Time = 31 Tad
    AD1CON3bits.ADCS = 0x2;     // Conversion Clock = 2 * Tcy

    SETBIT(AD1CON1bits.ADON);
}

void touch_select_dim(uint8_t dim) {
    if (dim == TOUCH_X) {
        CLEARBIT(PORTEbits.RE1);
        SETBIT(PORTEbits.RE2);
        SETBIT(PORTEbits.RE3);
        
        AD1CHS0bits.CH0SA = 15; 
        
    } else {
        SETBIT(PORTEbits.RE1);
        CLEARBIT(PORTEbits.RE2);
        CLEARBIT(PORTEbits.RE3);
        
        AD1CHS0bits.CH0SA = 9; 
    }
    
    __delay_ms(10);
}

uint16_t touch_read(void) {
    SETBIT(AD1CON1bits.SAMP);
    
    // Wait for conversion to complete
    while (!AD1CON1bits.DONE);
    
    // Clear Done bit
    CLEARBIT(AD1CON1bits.DONE);
    
    // Return result
    return ADC1BUF0;
}
/*
 * main loop
 */

void main_loop()
{
    // print assignment information
    lcd_printf("Lab05: Touchscreen &\r\n");
    lcd_printf("       Servos");
    lcd_locate(0, 2);
    lcd_printf("Group: 8");
    
    // initialize touchscreen
    touch_initialize();
    // initialize servos
    servo_initialize();

    int state = 0;
    uint16_t x_pos = 0;
    uint16_t y_pos = 0;
    int i = 0;
    
    while(TRUE) {
        switch(state) {
            case 0: // Front-Left (Min X, Min Y)
                servo_set_duty(SERVO_X, PWM_MIN_US);
                servo_set_duty(SERVO_Y, PWM_MIN_US);
                lcd_locate(0, 3);
                lcd_printf("Target: Back-Left ");
                break;
            case 3: // Front-Right (Max X, Min Y)
                servo_set_duty(SERVO_X, PWM_MAX_US);
                servo_set_duty(SERVO_Y, PWM_MIN_US);
                lcd_locate(0, 3);
                lcd_printf("Target: Front-Left");
                break;
            case 2: // Back-Right (Max X, Max Y)
                servo_set_duty(SERVO_X, PWM_MAX_US);
                servo_set_duty(SERVO_Y, PWM_MAX_US);
                lcd_locate(0, 3);
                lcd_printf("Target: Fornt-Right ");
                break;
            case 1: // Back-Left (Min X, Max Y)
                servo_set_duty(SERVO_X, PWM_MIN_US);
                servo_set_duty(SERVO_Y, PWM_MAX_US);
                lcd_locate(0, 3);
                lcd_printf("Target: Back-Right  ");
                break;
        }

        // We iterate 50 times * 100ms = 5000ms = 5s
        for(i = 0; i < 50; i++) {
            touch_select_dim(TOUCH_X); // Takes 10ms
            x_pos = touch_read();
            
            touch_select_dim(TOUCH_Y); // Takes 10ms
            y_pos = touch_read();
            
            lcd_locate(0, 5);
            lcd_printf("X: %u    ", x_pos);
            lcd_locate(0, 6);
            lcd_printf("Y: %u    ", y_pos);
            
            // Remaining delay to reach ~100ms cycle
            // (SelectDim takes 10ms * 2 = 20ms)
            __delay_ms(80);
        }

        // Increment state (0->1->2->3->0)
        state++;
        if(state > 3) state = 0;
        
    }
}
