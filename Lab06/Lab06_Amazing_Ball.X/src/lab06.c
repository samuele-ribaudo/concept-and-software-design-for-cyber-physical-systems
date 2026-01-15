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



/*
 * Common Definitions
 */
#define TCKPS_1   0x00
#define TCKPS_8   0x01
#define TCKPS_64  0x02
#define TCKPS_256 0x03



/*
 * Global Variables
 */



/*
 * Timer Code
 */



/*
 * Servo Code
 */



/*
 * Touch screen code
 */



/*
 * PD Controller
 */



/*
 * Butterworth Filter N=1, Cutoff 3 Hz, sampling @ 50 Hz
 */



/*
 * main loop
 */
void main_loop()
{
    // print assignment information
    lcd_printf("Lab06: Amazing Ball");
    lcd_locate(0, 1);
    lcd_printf("Group: GroupName");
    lcd_locate(0, 2);
    
    
    
    while(TRUE) {
        
    }
}
