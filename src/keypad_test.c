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

void GPIO_porta_handler() {
    uint8_t keypressed = keypad_check_key();
    GPIO_config_t led;
    led.GPIOx = GPIOF;
    led.GPIO_pin_num = 2;
    led.GPIO_pin_dir = GPIO_OUT;
    led.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    GPIO_init(&led);
    GPIO_write_pin(&led, GPIO_PIN_HIGH);
    delay_ms(1000);
    GPIO_write_pin(&led, GPIO_PIN_LOW);
    GPIO_PORTA_ICR_R |= 0x1C;
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
