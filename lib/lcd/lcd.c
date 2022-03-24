/*
 * lcd.c
 *
 *  Created on: Mar. 24, 2022
 *      Author: ben
 */

#include "lcd.h"

static SPI_config_t lcd;
static uint8_t lcd_ss_port;
static uint8_t lcd_ss_pin;

void LCD_init(uint8_t SPIx, uint8_t ss_port, uint8_t ss_pin) {
    lcd.SPIx = SPIx;
    lcd.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    lcd.SPI_cpol = SPI_CPOL_LOW;
    lcd.SPI_cpha = SPI_CPHA_LOW;
    lcd.SPI_speed = SPI_SCLK_SPEED_DIV64;
    lcd.SPI_dss = SPI_DSS_8BITS;

    SPI_init(&lcd);
    SPI_ss_init(ss_port, ss_pin);
    lcd_ss_port = ss_port;
    lcd_ss_pin = ss_pin;
}

void LCD_write_setting(uint8_t command) {
    SPI_ss_control(lcd_ss_port, lcd_ss_pin, SPI_SS_LOW);
    SPI_write_byte(&lcd, '|', lcd_ss_port, lcd_ss_pin, 0);
    SPI_write_byte(&lcd, command, lcd_ss_port, lcd_ss_pin, 1);
}

void LCD_display_string(char* string_to_display) {
    SPI_ss_control(lcd_ss_port, lcd_ss_pin, SPI_SS_LOW);
    SPI_write_string(&lcd, string_to_display, lcd_ss_port, lcd_ss_pin);
}

void LCD_move_cursor(uint8_t line, uint8_t position) {
    uint8_t commands[3] = {254, 0, '\0'};
    uint8_t position_command = 128;
    if (line != 0) {
        position_command += 64;
    }
    position_command += position;
    commands [1] = position_command;
    SPI_write_string(&lcd, commands, lcd_ss_port, lcd_ss_pin);
}

