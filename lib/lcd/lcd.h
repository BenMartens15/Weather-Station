/*
 * lcd.h
 *
 *  Created on: Mar. 24, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_LCD_LCD_H_
#define REPO_LIB_LCD_LCD_H_

#include <stdint.h>
#include "repo/lib/drivers/spi_driver.h"

void LCD_init(uint8_t SPIx, uint8_t ss_port, uint8_t ss_pin);
void LCD_write_setting(uint8_t command);
void LCD_display_string(char* string_to_display);
void LCD_move_cursor(uint8_t line, uint8_t position);
void LCD_change_backlight(uint8_t color);
void LCD_clear();

#endif /* REPO_LIB_LCD_LCD_H_ */
