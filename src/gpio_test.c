/*
 * gpio_test.c
 *
 *  Created on: Mar. 21, 2022
 *      Author: ben
 */

#include "repo/lib/drivers/gpio_driver.h"

void delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

int main() {
    GPIO_config_t GPIOF2;
    GPIO_config_t GPIOF4;

    GPIOF2.GPIOx = GPIOF;
    GPIOF2.GPIO_pin_num = GPIO_PIN_NUM_2;
    GPIOF2.GPIO_pin_dir = GPIO_OUT;

    GPIOF4.GPIOx = GPIOF;
    GPIOF4.GPIO_pin_num = GPIO_PIN_NUM_4;
    GPIOF4.GPIO_pin_dir = GPIO_IN;
    GPIOF4.GPIO_pu_pd = GPIO_PIN_PU;

    GPIO_init(&GPIOF2);
    GPIO_init(&GPIOF4);
    GPIO_interrupt_config(&GPIOF4, GPIO_LEVEL_SENSITIVE, GPIO_LOW_FALLING_TRIGGER);

    while(1);
}

void GPIO_porta_handler() {

}

void GPIO_portb_handler() {

}

void GPIO_portc_handler() {

}

void GPIO_portd_handler() {

}

void GPIO_porte_handler() {

}

void GPIO_portf_handler() {

}
