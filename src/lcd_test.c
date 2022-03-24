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
    LCD_display_string("B");
    SPI_delay(1000);
    LCD_move_cursor(0, 2);
    LCD_display_string("e");
    SPI_delay(1000);
    LCD_move_cursor(0, 4);
    LCD_display_string("n");
    SPI_delay(1000);
    LCD_move_cursor(1, 0);
    LCD_display_string("B");
    SPI_delay(1000);
    LCD_move_cursor(1, 2);
    LCD_display_string("e");
    SPI_delay(1000);
    LCD_move_cursor(1, 4);
    LCD_display_string("n");
    SPI_delay(1000);
    SPI_delay(250);
}


