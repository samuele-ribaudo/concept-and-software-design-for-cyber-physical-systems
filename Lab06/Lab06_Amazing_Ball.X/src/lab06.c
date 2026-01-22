#include "lab06.h"

#include <xc.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL
#include <libpic30.h>
#include <stdint.h>

#include <math.h>

#include "types.h"
#include "lcd.h"
#include "led.h"


/*
 * Parameter
 */
// to tune before launching the program
#define X_LEVELED_US 1557
#define Y_LEVELED_US 1525
#define MIN_X 71
#define MAX_X 711
#define MIN_Y 91
#define MAX_Y 696

#define PWM_MIN_US 1000
#define PWM_MID_US 1500
#define PWM_MAX_US 2000
#define PWM_CYC_US 20000

#define SERVO_X 0
#define SERVO_Y 1
#define TOUCH_X 0
#define TOUCH_Y 1
#define TMR1_PERIOD 1999
#define TMR2_PERIOD 3999

#define CIRCLE_RADIUS 100.0f // Radius
#define CIRCLE_SPEED 0.02f   // Angular speed (radians per tick)
#define CENTER_X (MIN_X + MAX_X) / 2.0f
#define CENTER_Y (MIN_Y + MAX_Y) / 2.0f





/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03


/* Function Prototypes */
void touch_select_dim(uint8_t dim);
uint16_t touch_read(void);
void servo_set_duty(uint8_t servo, uint16_t duty_us);
float butterworth_filter_x(float x_input);
float butterworth_filter_y(float y_input);
void pd_controller(void);

/*
 * Global Variables
 */

// Storage for raw and filtered values
volatile float cur_x_raw = 0;
volatile float cur_y_raw = 0;
volatile float cur_x_filtered = 0;
volatile float cur_y_filtered = 0;

// Desired Position (Center of screen by default)
volatile float setpoint_x = (MIN_X + MAX_X) / 2.0f;
volatile float setpoint_y = (MIN_Y + MAX_Y) / 2.0f;

// deadline misses
volatile int deadline_misses = 0;
volatile int trigger_100Hz = 0;
volatile int current_dim = TOUCH_X;



/*
 * Timer Code
 */
void timer1_initialize(void){
    T1CONbits.TON = 0;      // Disable Timer
    T1CONbits.TCS = 0;      // Select internal instruction cycle clock
    T1CONbits.TGATE = 0;    // Disable Gated Timer mode
    T1CONbits.TCKPS = TCKPS_64; // Select 1:64 Prescaler

    TMR1 = 0x00;            // Clear timer register
    PR1 = TMR1_PERIOD;             // Load the period value (10ms @ 64 prescaler)

    IPC0bits.T1IP = 0x01;   // Set Timer1 Interrupt Priority Level
    IFS0bits.T1IF = 0;      // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;      // Enable Timer1 interrupt

    T1CONbits.TON = 1;      // Start Timer
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void){
    IFS0bits.T1IF = 0; // clear flag
    if(trigger_100Hz == 1) deadline_misses++;
    trigger_100Hz = 1; // signal main loop to run
}




/*
 * Servo Code
 */
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
    if (duty_us < 1000) duty_us = 1000;
    if (duty_us > 2000) duty_us = 2000;

    // Convert microseconds to ticks (5us per tick with 1:64 prescaler)
    uint16_t ticks = duty_us / 5;
    uint16_t inverted_duty = TMR2_PERIOD - ticks;

    if (servo == SERVO_X) {
        OC8RS = inverted_duty;
    } else if (servo == SERVO_Y) {
        OC7RS = inverted_duty;
    }
}




/*
 * Touch screen code
 */
void touch_initialize(void) {
    CLEARBIT(AD1CON1bits.ADON);
    
    // Configure Ports
    CLEARBIT(TRISEbits.TRISE1); // E1
    CLEARBIT(TRISEbits.TRISE2); // E2
    CLEARBIT(TRISEbits.TRISE3); // E3
    SETBIT(TRISBbits.TRISB15);  // Input for AN15
    SETBIT(TRISBbits.TRISB9);   // Input for AN9

    // Configure AD1PCFGL (0 = Analog, 1 = Digital)
    CLEARBIT(AD1PCFGLbits.PCFG15); // AN15 Analog
    CLEARBIT(AD1PCFGLbits.PCFG9);  // AN9 Analog

    // Configure ADC
    AD1CON1bits.FORM = 0; // Integer Output
    AD1CON1bits.SSRC = 0x7; // Auto-Conversion
    AD1CON1bits.ASAM = 0; // Manual Sampling
    AD1CON1bits.AD12B = 0; // 10-bit mode
    AD1CON2 = 0; // MUX A only
    AD1CON3bits.ADRC = 0; // System Clock
    AD1CON3bits.SAMC = 0x1F; // Sample Time = 31 Tad
    AD1CON3bits.ADCS = 0x2; // Conversion Clock = 2 * Tcy

    SETBIT(AD1CON1bits.ADON);
}

void touch_select_dim(uint8_t dim) {
    if (dim == TOUCH_X) {
        CLEARBIT(PORTEbits.RE1);
        Nop();
        SETBIT(PORTEbits.RE2);
        Nop();
        SETBIT(PORTEbits.RE3);
        Nop();
        AD1CHS0bits.CH0SA = 15; // AN15
    } else {
        SETBIT(PORTEbits.RE1);
        Nop();
        CLEARBIT(PORTEbits.RE2);
        Nop();
        CLEARBIT(PORTEbits.RE3);
        Nop();
        AD1CHS0bits.CH0SA = 9;  // AN9
    }
}

uint16_t touch_read(void) {
    SETBIT(AD1CON1bits.SAMP);
    while (!AD1CON1bits.DONE);
    CLEARBIT(AD1CON1bits.DONE);
    return ADC1BUF0;
}



/*
 * PD Controller
 */
float Kp_x = 0.35;                                                                                                       f; 
float Kd_x = 7.0f; 
float Kp_y = 0.4f; 
float Kd_y = 8.5f; 

void pd_controller(void) {
    static float error_x_old = 0;
    static float error_y_old = 0;
    
    float error_x, error_y;
    float deriv_x, deriv_y;
    float out_x, out_y;
    float pid_out_x, pid_out_y;
    
    // --- X AXIS ---
    // 1. Calculate Error
    error_x = setpoint_x - cur_x_filtered;
    
    // 2. Calculate Derivative (Change in error)
    deriv_x = error_x - error_x_old;
    
    // 3. Calculate PID Term
    pid_out_x = (Kp_x * error_x) + (Kd_x * deriv_x);
    
    // 4. Calculate Final Servo Output (Base + PID)
    out_x = X_LEVELED_US + pid_out_x;
    
    // 5. Save history
    error_x_old = error_x;
    
    
    // --- Y AXIS ---
    error_y = setpoint_y - cur_y_filtered;
    deriv_y = error_y - error_y_old;
    
    pid_out_y = (Kp_y * error_y) + (Kd_y * deriv_y);
    out_y = Y_LEVELED_US + pid_out_y;
    
    error_y_old = error_y;

    // --- APPLY TO SERVOS ---
    // Cast to uint16_t (and implicitly clamp in servo_set_duty)
    servo_set_duty(SERVO_X, (uint16_t)out_x);
    servo_set_duty(SERVO_Y, (uint16_t)out_y);
}




/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */
#define B1 0.1602f
#define B2 0.1602f
#define A2 -0.6796f

float butterworth_filter_x(float x_input) {
    static float x_old = 0; // Previous input x[n-1]
    static float y_old = 0; // Previous output y[n-1]
    float y_new;

    // y[n] = b1*x[n] + b2*x[n-1] - a2*y[n-1]
    y_new = B1 * x_input + B2 * x_old - A2 * y_old;

    // Update state
    x_old = x_input;
    y_old = y_new;

    return y_new;
}

float butterworth_filter_y(float y_input) {
    static float x_old = 0; 
    static float y_old = 0; 
    float y_new;

    y_new = B1 * y_input + B2 * x_old - A2 * y_old;

    x_old = y_input;
    y_old = y_new;

    return y_new;
}




/*
 * main loop
*/
void main_loop()
{
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: 8");

    // Init Hardware
    servo_initialize();
    touch_initialize();
    
    // Prep Touchscreen
    touch_select_dim(TOUCH_X);
    current_dim = TOUCH_X;
    
    // Start Timer (Interrupts begin)
    timer1_initialize();
    
    int loop_counter = 0; // to track 50Hz and 5Hz zones
    
    while(TRUE) {
        if(trigger_100Hz){
            trigger_100Hz = 0; // clear flag to acknowledge
            loop_counter++;

            uint16_t val = touch_read();

            if(current_dim == TOUCH_X){
                cur_x_raw = val;
                cur_x_filtered = butterworth_filter_x(cur_x_raw);

                // switch to Y for next cycle
                touch_select_dim(TOUCH_Y);
                current_dim = TOUCH_Y;
            }else{
                cur_y_raw = val;
                cur_y_filtered = butterworth_filter_y(cur_y_raw);

                // switch to X for next cycle
                touch_select_dim(TOUCH_X);
                current_dim = TOUCH_X;

                // update circle setpoint
                static float theta = 0;
                theta += CIRCLE_SPEED;
                if (theta > 6.283f) theta -= 6.283f;
                
                setpoint_x = CENTER_X + (CIRCLE_RADIUS * cosf(theta));
                setpoint_y = CENTER_Y + (CIRCLE_RADIUS * sinf(theta));
                
                // run the PD controller
                pd_controller();
            }

            // 5Hz code
            if(loop_counter >= 20){
                lcd_locate(0,3);
                lcd_printf("Deadline misses: %d", deadline_misses);
                lcd_locate(0,5);
                lcd_printf("X: %4.0f Y:%4.0f", cur_x_filtered, cur_y_filtered);
                loop_counter = 0;
            }
        }
    }
}


/*
 * main loop for Calibration/Setup

void main_loop() {
    // 1. Initialize Hardware
    servo_initialize();
    touch_initialize();
    
    // 2. Set Servos to a fixed position (calibration center)
    servo_set_duty(SERVO_X, 1557);
    servo_set_duty(SERVO_Y, 1525);
    
    lcd_printf("Calibration Mode");
    
    // 3. Variables for tracking Min/Max
    uint16_t raw_x = 0;
    uint16_t raw_y = 0;
    
    // Initialize Min/Max to opposite extremes
    uint16_t min_x = 1023, max_x = 0;
    uint16_t min_y = 1023, max_y = 0;

    // 4. Loop
    while(TRUE) {
        // --- READ X ---
        touch_select_dim(TOUCH_X);
        __delay_ms(10); // Wait for settle
        raw_x = touch_read();
        
        // Update Min/Max X
        if (raw_x < min_x && raw_x > 0) min_x = raw_x; // Ignore 0 noise
        if (raw_x > max_x) max_x = raw_x;

        // --- READ Y ---
        touch_select_dim(TOUCH_Y);
        __delay_ms(10); // Wait for settle
        raw_y = touch_read();
        
        // Update Min/Max Y
        if (raw_y < min_y && raw_y > 0) min_y = raw_y; // Ignore 0 noise
        if (raw_y > max_y) max_y = raw_y;

        // --- DISPLAY ---
        // Row 1: Current Values
        lcd_locate(0, 1);
        lcd_printf("X:%4u Y:%4u", raw_x, raw_y);
        
        // Row 2: X Min/Max
        lcd_locate(0, 2);
        lcd_printf("Xm:%3u XM:%3u", min_x, max_x);

        // Row 3: Y Min/Max
        lcd_locate(0, 3);
        lcd_printf("Ym:%3u YM:%3u", min_y, max_y);
        
        // Small delay to make screen readable
        __delay_ms(100);
    }
}


 */