/*
 * stepper_motor.h
 *
 *  Created on: Apr. 7, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_STEPPER_MOTOR_STEPPER_MOTOR_H_
#define REPO_LIB_STEPPER_MOTOR_STEPPER_MOTOR_H_

#include "repo/lib/drivers/gpio_driver.h"

void stepper_init(uint8_t steps_per_revolution, uint8_t in1_port, uint8_t in1_pin, uint8_t in2_port, uint8_t in2_pin, uint8_t in3_port, uint8_t in3_pin, uint8_t in4_port, uint8_t in4_pin);
void stepper_rotate(uint8_t direction, uint8_t speed, uint16_t steps_to_rotate);

/*
 * possibilities for direction parameter in rotate_continuous
 */
#define DIRECTION_CLOCKWISE             0
#define DIRECTION_COUNTER_CLOCKWISE     1

#endif /* REPO_LIB_STEPPER_MOTOR_STEPPER_MOTOR_H_ */
