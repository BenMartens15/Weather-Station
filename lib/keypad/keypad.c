/*
 * keypad.c
 *
 *  Created on: Mar. 22, 2022
 *      Author: ben
 */

#include "keypad.h"
#include <string.h>

static GPIO_config_t keypad_columns;
static GPIO_config_t keypad_rows;
static uint8_t column_port;
static uint8_t column_pins[3];
static uint8_t row_port;
static uint8_t row_pins[4];

void keypad_init(uint8_t c_port, uint8_t r_port, uint8_t* c_pins, uint8_t* r_pins) {
    column_port = c_port;
    memcpy(column_pins, c_pins, sizeof(column_pins));
    row_port = r_port;
    memcpy(row_pins, r_pins, sizeof(row_pins));

    // configure the pins to read keypad columns
    keypad_columns.GPIO_pin_dir = GPIO_IN;
    keypad_columns.GPIO_pu_pd = GPIO_PIN_PU;
    keypad_columns.GPIOx = column_port;

    keypad_columns.GPIO_pin_num = column_pins[0];
    GPIO_init(&keypad_columns);
    GPIO_interrupt_config(&keypad_columns, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    keypad_columns.GPIO_pin_num = column_pins[1];
    GPIO_init(&keypad_columns);
    GPIO_interrupt_config(&keypad_columns, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    keypad_columns.GPIO_pin_num = column_pins[2];
    GPIO_init(&keypad_columns);
    GPIO_interrupt_config(&keypad_columns, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    // configure the pins to drive keypad rows
    keypad_rows.GPIO_pin_dir = GPIO_OUT;
    keypad_rows.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    keypad_rows.GPIOx = row_port;

    keypad_rows.GPIO_pin_num = row_pins[0];
    GPIO_init(&keypad_rows);
    GPIO_write_pin(&keypad_rows, GPIO_PIN_LOW);

    keypad_rows.GPIO_pin_num = row_pins[1];
    GPIO_init(&keypad_rows);
    GPIO_write_pin(&keypad_rows, GPIO_PIN_LOW);

    keypad_rows.GPIO_pin_num = row_pins[2];
    GPIO_init(&keypad_rows);
    GPIO_write_pin(&keypad_rows, GPIO_PIN_LOW);

    keypad_rows.GPIO_pin_num = row_pins[3];
    GPIO_init(&keypad_rows);
    GPIO_write_pin(&keypad_rows, GPIO_PIN_LOW);
}

uint8_t keypad_check_key() {
    uint8_t row_pressed;
    uint8_t column_pressed;
    uint8_t test_row;
    for (test_row = 0; test_row < 4; test_row++) { // make each row high
        keypad_rows.GPIO_pin_num = row_pins[test_row];
        GPIO_write_pin(&keypad_rows, GPIO_PIN_HIGH);
    }
    for (test_row = 0; test_row < 4; test_row++) { // write each row low and see if a button has been pressed
        keypad_rows.GPIO_pin_num = row_pins[test_row];
        GPIO_write_pin(&keypad_rows, GPIO_PIN_LOW);
        uint8_t test_column;
        for (test_column = 0; test_column < 3; test_column++) {
            keypad_columns.GPIO_pin_num = column_pins[test_column];
            if (GPIO_read_pin(&keypad_columns) == GPIO_PIN_LOW) {
                row_pressed = test_row;
                column_pressed = test_column;
            }
        }
        GPIO_write_pin(&keypad_rows, GPIO_PIN_HIGH);
    }

    if (row_pressed == 0) {
        if (column_pressed == 0) {
            return 1;
        }
        else if (column_pressed == 1) {
            return 2;
        }
        else if (column_pressed == 2) {
            return 3;
        }
    }
    else if (row_pressed == 1) {
        if (column_pressed == 0) {
            return 4;
        }
        else if (column_pressed == 1) {
            return 5;
        }
        else if (column_pressed == 2) {
            return 6;
        }
    }
    else if (row_pressed == 2) {
        if (column_pressed == 0) {
            return 7;
        }
        else if (column_pressed == 1) {
            return 8;
        }
        else if (column_pressed == 2) {
            return 9;
        }
    }
    else if (row_pressed == 3) {
        if (column_pressed == 0) {
            return 10;
        }
        else if (column_pressed == 1) {
            return 11;
        }
        else if (column_pressed == 2) {
            return 12;
        }
    }
    return 13;
}
