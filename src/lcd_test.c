/*
 * lcd_test.c
 *
 *  Created on: Mar. 24, 2022
 *      Author: ben
 */

#include "repo/lib/lcd/lcd.h"
#include <stdio.h>

int main() {
    LCD_init(SPI3, SPI_SS_PORT_D, SPI_SS_PIN_1);
    SPI_delay(1000);
    LCD_clear();
    LCD_write_setting('/'); // disable system messages
    LCD_change_backlight('r');
    LCD_display_string("Red");
    SPI_delay(1000);
    LCD_clear();
    LCD_change_backlight('g');
    LCD_display_string("Green");
    SPI_delay(1000);
    LCD_clear();
    LCD_change_backlight('b');
    LCD_display_string("Blue");
    SPI_delay(1000);
    LCD_clear();
    LCD_change_backlight('w');
    LCD_display_string("White");
    SPI_delay(1000);
    LCD_clear();
}


