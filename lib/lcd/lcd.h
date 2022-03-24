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

void LCD_init(uint8_t SPIx);
void LCD_display_char(uint8_t character_to_display);
void LCD_display(uint8_t* string_to_display);

#endif /* REPO_LIB_LCD_LCD_H_ */
