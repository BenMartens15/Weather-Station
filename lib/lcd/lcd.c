/*
 * lcd.c
 *
 *  Created on: Mar. 24, 2022
 *      Author: ben
 */

#include "lcd.h"

static SPI_config_t lcd;

void LCD_init(uint8_t SPIx, uint8_t ss_port, uint8_t ss_pin) {
    lcd.SPIx = SPIx;
    lcd.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    lcd.SPI_cpol = SPI_CPOL_LOW;
    lcd.SPI_cpha = SPI_CPHA_LOW;
    lcd.SPI_speed = SPI_SCLK_SPEED_DIV64;
    lcd.SPI_dss = SPI_DSS_8BITS;

    SPI_init(&lcd);
    SPI_ss_init(ss_port, ss_pin);
}

void LCD_display_char(uint8_t character_to_display) {

}

void LCD_display(uint8_t* string_to_display) {

}
