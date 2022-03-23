/*
 * keypad.c
 *
 *  Created on: Mar. 22, 2022
 *      Author: ben
 */

#include "keypad.h"
#include <string.h>

static GPIO_config_t keypad;
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
    keypad.GPIO_pin_dir = GPIO_IN;
    keypad.GPIO_pu_pd = GPIO_PIN_PU;
    keypad.GPIOx = column_port;

    keypad.GPIO_pin_num = column_pins[0];
    GPIO_init(&keypad);
    GPIO_interrupt_config(&keypad, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    keypad.GPIO_pin_num = column_pins[1];
    GPIO_init(&keypad);
    GPIO_interrupt_config(&keypad, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    keypad.GPIO_pin_num = column_pins[2];
    GPIO_init(&keypad);
    GPIO_interrupt_config(&keypad, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    // configure the pins to drive keypad rows
    keypad.GPIO_pin_dir = GPIO_OUT;
    keypad.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    keypad.GPIOx = row_port;

    keypad.GPIO_pin_num = row_pins[0];
    GPIO_init(&keypad);

    keypad.GPIO_pin_num = row_pins[1];
    GPIO_init(&keypad);

    keypad.GPIO_pin_num = row_pins[2];
    GPIO_init(&keypad);

    keypad.GPIO_pin_num = row_pins[3];
    GPIO_init(&keypad);
}

uint8_t keypad_check_key() {
    if (column_port == GPIOA) {

    }
    else if (column_port == GPIOB) {

    }
    else if (column_port == GPIOC) {

    }
    else if (column_port == GPIOD) {

    }
    else if (column_port == GPIOE) {

    }
    else if (column_port == GPIOF) {

    }
}

void GPIO_porta_handler() {
    keypad_check_key();
}

void GPIO_portb_handler() {
    keypad_check_key();
}

void GPIO_portc_handler() {
    keypad_check_key();
}

void GPIO_portd_handler() {
    keypad_check_key();
}

void GPIO_porte_handler() {
    keypad_check_key();
}

void GPIO_portf_handler() {
    keypad_check_key();
}
