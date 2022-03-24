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
    LCD_move_cursor(0, 5);
    LCD_display_string("12:34am");
    LCD_move_cursor(1, 0);
    LCD_display_string("Temp: 22.7 C");
    SPI_delay(5000);
    LCD_move_cursor(0, 5);
    LCD_display_string("12:35am");
    SPI_delay(250);
}


