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
    LCD_write_setting('-');
    LCD_write_setting('/'); // disable system messages
    LCD_change_backlight('r');
    SPI_delay(1000);
    LCD_change_backlight('g');
    SPI_delay(1000);
    LCD_change_backlight('b');
    SPI_delay(1000);
    LCD_change_backlight('w');
}


