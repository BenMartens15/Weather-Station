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
static uint8_t keypad_key_pressed;

static void debounce() { // delay for 10ms to debounce the keypad buttons
    int i, j;
    for(i = 0 ; i < 50; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

uint8_t get_keypad_key_pressed() {
    return keypad_key_pressed;
}

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
    debounce();
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

    for (test_row = 0; test_row < 4; test_row++) { // make each row low again
        keypad_rows.GPIO_pin_num = row_pins[test_row];
        GPIO_write_pin(&keypad_rows, GPIO_PIN_LOW);
    }

    debounce(); // not really sure why this delay is needed, but this logic block always returns 1 if the delay isn't here
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

void GPIO_porta_handler() {
    GPIO_PORTA_IM_R &= ~(1 << column_pins[0]); // mask the interrupts to prevent firing multiple times
    GPIO_PORTA_IM_R &= ~(1 << column_pins[1]);
    GPIO_PORTA_IM_R &= ~(1 << column_pins[2]);
    keypad_key_pressed = keypad_check_key();
    GPIO_PORTA_ICR_R |= (1 << column_pins[0]);
    GPIO_PORTA_ICR_R |= (1 << column_pins[1]);
    GPIO_PORTA_ICR_R |= (1 << column_pins[2]);
    GPIO_PORTA_IM_R |= (1 << column_pins[0]); // unmask the interrupts
    GPIO_PORTA_IM_R |= (1 << column_pins[1]);
    GPIO_PORTA_IM_R |= (1 << column_pins[2]);
}

void GPIO_portb_handler() {
    GPIO_PORTB_IM_R &= ~(1 << column_pins[0]); // mask the interrupts to prevent firing multiple times
    GPIO_PORTB_IM_R &= ~(1 << column_pins[1]);
    GPIO_PORTB_IM_R &= ~(1 << column_pins[2]);
    keypad_key_pressed = keypad_check_key();
    GPIO_PORTB_ICR_R |= (1 << column_pins[0]);
    GPIO_PORTB_ICR_R |= (1 << column_pins[1]);
    GPIO_PORTB_ICR_R |= (1 << column_pins[2]);
    GPIO_PORTB_IM_R |= (1 << column_pins[0]); // unmask the interrupts
    GPIO_PORTB_IM_R |= (1 << column_pins[1]);
    GPIO_PORTB_IM_R |= (1 << column_pins[2]);
}

void GPIO_portc_handler() {
    GPIO_PORTC_IM_R &= ~(1 << column_pins[0]); // mask the interrupts to prevent firing multiple times
    GPIO_PORTC_IM_R &= ~(1 << column_pins[1]);
    GPIO_PORTC_IM_R &= ~(1 << column_pins[2]);
    keypad_key_pressed = keypad_check_key();
    GPIO_PORTC_ICR_R |= (1 << column_pins[0]);
    GPIO_PORTC_ICR_R |= (1 << column_pins[1]);
    GPIO_PORTC_ICR_R |= (1 << column_pins[2]);
    GPIO_PORTC_IM_R |= (1 << column_pins[0]); // unmask the interrupts
    GPIO_PORTC_IM_R |= (1 << column_pins[1]);
    GPIO_PORTC_IM_R |= (1 << column_pins[2]);
}

void GPIO_portd_handler() {
    GPIO_PORTD_IM_R &= ~(1 << column_pins[0]); // mask the interrupts to prevent firing multiple times
    GPIO_PORTD_IM_R &= ~(1 << column_pins[1]);
    GPIO_PORTD_IM_R &= ~(1 << column_pins[2]);
    keypad_key_pressed = keypad_check_key();
    GPIO_PORTD_ICR_R |= (1 << column_pins[0]);
    GPIO_PORTD_ICR_R |= (1 << column_pins[1]);
    GPIO_PORTD_ICR_R |= (1 << column_pins[2]);
    GPIO_PORTD_IM_R |= (1 << column_pins[0]); // unmask the interrupts
    GPIO_PORTD_IM_R |= (1 << column_pins[1]);
    GPIO_PORTD_IM_R |= (1 << column_pins[2]);
}

void GPIO_porte_handler() {
    GPIO_PORTE_IM_R &= ~(1 << column_pins[0]); // mask the interrupts to prevent firing multiple times
    GPIO_PORTE_IM_R &= ~(1 << column_pins[1]);
    GPIO_PORTE_IM_R &= ~(1 << column_pins[2]);
    keypad_key_pressed = keypad_check_key();
    GPIO_PORTE_ICR_R |= (1 << column_pins[0]);
    GPIO_PORTE_ICR_R |= (1 << column_pins[1]);
    GPIO_PORTE_ICR_R |= (1 << column_pins[2]);
    GPIO_PORTE_IM_R |= (1 << column_pins[0]); // unmask the interrupts
    GPIO_PORTE_IM_R |= (1 << column_pins[1]);
    GPIO_PORTE_IM_R |= (1 << column_pins[2]);
}

void GPIO_portf_handler() {
    GPIO_PORTF_IM_R &= ~(1 << column_pins[0]); // mask the interrupts to prevent firing multiple times
    GPIO_PORTF_IM_R &= ~(1 << column_pins[1]);
    GPIO_PORTF_IM_R &= ~(1 << column_pins[2]);
    keypad_key_pressed = keypad_check_key();
    GPIO_PORTF_ICR_R |= (1 << column_pins[0]);
    GPIO_PORTF_ICR_R |= (1 << column_pins[1]);
    GPIO_PORTF_ICR_R |= (1 << column_pins[2]);
    GPIO_PORTF_IM_R |= (1 << column_pins[0]); // unmask the interrupts
    GPIO_PORTF_IM_R |= (1 << column_pins[1]);
    GPIO_PORTF_IM_R |= (1 << column_pins[2]);
}

