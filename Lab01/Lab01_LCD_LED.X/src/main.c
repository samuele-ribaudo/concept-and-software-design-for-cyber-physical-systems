#include <xc.h>
#include <p33Fxxxx.h>
//do not change the order of the following 2 definitions
#define FCY 12800000UL 
#include <libpic30.h>

#include "lcd.h"
#include "led.h"

/* Configuration of the Chip */
// Initial Oscillator Source Selection = Primary (XT, HS, EC) Oscillator with PLL
#pragma config FNOSC = PRIPLL
// Primary Oscillator Mode Select = XT Crystal Oscillator mode
#pragma config POSCMD = XT
// Watchdog Timer Enable = Watchdog Timer enabled/disabled by user software
// (LPRC can be disabled by clearing the SWDTEN bit in the RCON register)
#pragma config FWDTEN = OFF

int main() {
    //Init LCD and LEDs
    lcd_initialize();
    led_init();
	
    // Clear the Screen and reset the cursor
    lcd_clear();
    lcd_locate(0, 0);
    
    // Print Name
    lcd_printf("Group Members:\r");
    lcd_printf("* Samuele Ribaudo\r");
    lcd_printf("* Hong Yan Jun\r");
    lcd_printf("* Aditya Kotte\r");
    lcd_locate(0, 7);
    printf("Counter = 0");
    
    
    //Declare count and turn it into LED
    uint8_t count = 1;
    while (count <= 128) {
        
        //half a second delay
        __delay_ms(500);
        
        //print counter value
        lcd_locate(10, 7);
        printf("%d", count);
        
        //LED5 handling
        TOGGLELED(LED5_PORT);
        
        //LED4 handling
        if (count % 2 == 0) {
            TOGGLELED(LED4_PORT);
        } 
        
        //LED3 handling
        if (count % 4 == 0) {
            TOGGLELED(LED3_PORT);
        } 
        
        //LED2 handling
        if (count % 8 == 0) {
            TOGGLELED(LED2_PORT);
        } 
        
        //LED1 handling
        if (count % 16 == 0) {
            TOGGLELED(LED1_PORT);
        } 
        
        //increment counter value
        count++;
    }
    
    // Stop
    while(1)
        ;
}


