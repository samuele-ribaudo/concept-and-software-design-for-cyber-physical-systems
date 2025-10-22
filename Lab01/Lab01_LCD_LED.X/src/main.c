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
    // Initialize
    lcd_initialize();
    led_init();
    lcd_clear();

    gotoLine(1);
    lcd_printf("Aditya kotte");
    gotoLine(2);
    lcd_printf("Samuele Ribaudo");
    gotoLine(3);
    lcd_printf("Yan Jun Hong");
    

    // Set LED pins as outputs
    CLEARLED(LED1_TRIS);
    CLEARLED(LED2_TRIS);
    CLEARLED(LED3_TRIS);
    CLEARLED(LED4_TRIS);
    CLEARLED(LED5_TRIS);

    uint8_t count = 0;

    while (count < 128) {
        // Display number on LCD
        gotoLine(5);  // safer than 8
        lcd_clear_row(4);
        lcd_printf("Count: %d", count);

        // Set LEDs according to binary value of count
        // LED1 (bit 4)
        if (count & (1 << 4)) SETLED(LED1_PORT);
        else CLEARLED(LED1_PORT);

        // LED2 (bit 3)
        if (count & (1 << 3)) SETLED(LED2_PORT);
        else CLEARLED(LED2_PORT);

        // LED3 (bit 2)
        if (count & (1 << 2)) SETLED(LED3_PORT);
        else CLEARLED(LED3_PORT);

        // LED4 (bit 1)
        if (count & (1 << 1)) SETLED(LED4_PORT);
        else CLEARLED(LED4_PORT);

        // LED5 (bit 0)
        if (count & (1 << 0)) SETLED(LED5_PORT);
        else CLEARLED(LED5_PORT);

        count++;
        __delay_ms(500);  // 0,5 second delay
    }

    while (1) {
    }
}


