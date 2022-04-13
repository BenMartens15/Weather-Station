/*
 * keypad.h
 *
 *  Created on: Mar. 22, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_KEYPAD_KEYPAD_H_
#define REPO_LIB_KEYPAD_KEYPAD_H_

#include <stdint.h>
#include "repo/lib/drivers/gpio_driver.h"

// returns the value of a variable storing which key was pressed
uint8_t get_keypad_key_pressed();

// initialize the GPIOs for the keypad
void keypad_init(uint8_t column_port, uint8_t row_port, uint8_t* column_pins, uint8_t* row_pins);

// check which key has been pressed (called from the interrupt handlers)
uint8_t keypad_check_key();

void GPIO_porta_handler();
void GPIO_portb_handler();
void GPIO_portc_handler();
void GPIO_portd_handler();
void GPIO_porte_handler();
void GPIO_portf_handler();

#endif /* REPO_LIB_KEYPAD_KEYPAD_H_ */
