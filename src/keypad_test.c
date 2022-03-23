/*
 * keypad_test.c
 *
 *  Created on: Mar. 22, 2022
 *      Author: ben
 */

#include "repo/lib/keypad/keypad.h"

void delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

int main() {
    /*
     * DO NOT USE GPIOF2 FOR A COLUMN PIN - IT WILL CAUSE THE INTERRUPT TO ALWAYS BE TRIGGERED
     */

    uint8_t keypad_column_pins[] = {GPIO_PIN_NUM_4, GPIO_PIN_NUM_3, GPIO_PIN_NUM_2};
    uint8_t keypad_row_pins[] = {GPIO_PIN_NUM_4, GPIO_PIN_NUM_5, GPIO_PIN_NUM_6, GPIO_PIN_NUM_7};

    keypad_init(GPIOA, GPIOC, keypad_column_pins, keypad_row_pins);
    while(1);
}

