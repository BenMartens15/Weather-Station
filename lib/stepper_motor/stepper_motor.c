/*
 * stepper_motor.c
 *
 *  Created on: Apr. 7, 2022
 *      Author: ben
 */

#include "stepper_motor.h"

static GPIO_config_t stepper_in1;
static GPIO_config_t stepper_in2;
static GPIO_config_t stepper_in3;
static GPIO_config_t stepper_in4;

static void stepper_delay(uint16_t time_ms) {
    uint16_t i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

void stepper_init(uint8_t steps_per_revolution, uint8_t in1_port, uint8_t in1_pin, uint8_t in2_port, uint8_t in2_pin, uint8_t in3_port, uint8_t in3_pin, uint8_t in4_port, uint8_t in4_pin) {
    stepper_in1.GPIOx = in1_port;
    stepper_in1.GPIO_pin_dir = GPIO_OUT;
    stepper_in1.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    stepper_in1.GPIO_pin_num = in1_pin;
    GPIO_init(&stepper_in1);

    stepper_in2.GPIOx = in2_port;
    stepper_in2.GPIO_pin_dir = GPIO_OUT;
    stepper_in2.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    stepper_in2.GPIO_pin_num = in2_pin;
    GPIO_init(&stepper_in2);

    // I swapped the ports and pins of stepper_in3 and stepper_in4 cause I accidently mixed them when I soldered the motor driver board
    stepper_in3.GPIOx = in4_port;
    stepper_in3.GPIO_pin_dir = GPIO_OUT;
    stepper_in3.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    stepper_in3.GPIO_pin_num = in4_pin;
    GPIO_init(&stepper_in3);

    stepper_in4.GPIOx = in3_port;
    stepper_in4.GPIO_pin_dir = GPIO_OUT;
    stepper_in4.GPIO_pu_pd = GPIO_PIN_NO_PUPD;
    stepper_in4.GPIO_pin_num = in3_pin;
    GPIO_init(&stepper_in4);

}

void stepper_rotate(uint8_t direction, uint8_t speed, uint16_t steps_to_rotate) {
    uint16_t steps;

    if (direction == DIRECTION_CLOCKWISE) {
        for(steps = 0; steps < steps_to_rotate / 4; steps++) {
            GPIO_write_pin(&stepper_in1, GPIO_PIN_HIGH);
            GPIO_write_pin(&stepper_in2, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in1, GPIO_PIN_LOW);
            GPIO_write_pin(&stepper_in3, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in2, GPIO_PIN_LOW);
            GPIO_write_pin(&stepper_in4, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in3, GPIO_PIN_LOW);
            GPIO_write_pin(&stepper_in1, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in4, GPIO_PIN_LOW);
        }
    }
    else {
        for(steps = 0; steps < steps_to_rotate / 4; steps++) {
            GPIO_write_pin(&stepper_in4, GPIO_PIN_HIGH);
            GPIO_write_pin(&stepper_in3, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in4, GPIO_PIN_LOW);
            GPIO_write_pin(&stepper_in2, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in3, GPIO_PIN_LOW);
            GPIO_write_pin(&stepper_in1, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in2, GPIO_PIN_LOW);
            GPIO_write_pin(&stepper_in4, GPIO_PIN_HIGH);
            stepper_delay(speed);
            GPIO_write_pin(&stepper_in1, GPIO_PIN_LOW);
        }
    }
}

