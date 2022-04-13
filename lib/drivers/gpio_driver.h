/*
 * gpio_driver.h
 *
 *  Created on: Mar. 21, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_DRIVERS_GPIO_DRIVER_H_
#define REPO_LIB_DRIVERS_GPIO_DRIVER_H_

#include <stdint.h>
#include "repo/lib/drivers/tm4c123gh6pm.h"

// GPIO pin configuration structure
typedef struct {
    uint8_t GPIOx; // GPIO port
    uint8_t GPIO_pin_num;
    uint8_t GPIO_pin_dir; // pin direction
    uint8_t GPIO_pu_pd; // pull-up/pull-down resistor control
}GPIO_config_t;

// enables or disables the peripheral clock associated with different GPIO ports
void GPIO_pclk_control(uint8_t GPIOx, uint8_t enable_disable);

// initializes a GPIO pin based on the GPIO configuration structure passed
void GPIO_init(GPIO_config_t *pGPIOConfig);

// GPIO pin read and write functions
uint8_t GPIO_read_pin(GPIO_config_t *pGPIOConfig);
void GPIO_write_pin(GPIO_config_t *pGPIOConfig, uint8_t high_low);

// configures interrupts for GPIO pins
void GPIO_interrupt_config(GPIO_config_t *pGPIConfig, uint8_t edge_level_sensitive, uint8_t edge_level_trigger);

#define GPIO_PCLK_ENABLE    1
#define GPIO_PCLK_DISABLE   0

#define GPIO_PIN_HIGH       1
#define GPIO_PIN_LOW        0

#define GPIO_EDGE_SENSITIVE     0
#define GPIO_LEVEL_SENSITIVE    1

#define GPIO_LOW_FALLING_TRIGGER    0
#define GPIO_HIGH_RISING_TRIGGER    1

/*************** Macros for elements of the GPIO configuration structure ***************/
/*
 * @GPIOx
 * GPIO port
 */
#define GPIOA               0
#define GPIOB               1
#define GPIOC               2
#define GPIOD               3
#define GPIOE               4
#define GPIOF               5

/*
 * @GPIO_pin_num
 * GPIO pin number
 */
#define GPIO_PIN_NUM_0      0
#define GPIO_PIN_NUM_1      1
#define GPIO_PIN_NUM_2      2
#define GPIO_PIN_NUM_3      3
#define GPIO_PIN_NUM_4      4
#define GPIO_PIN_NUM_5      5
#define GPIO_PIN_NUM_6      6
#define GPIO_PIN_NUM_7      7

/*
 * @GPIO_pin_dir
 * GPIO direction (input or output)
 */
#define GPIO_IN             0
#define GPIO_OUT            1
#define GPIO_ALTFN          2

/*
 * @GPIO_alt_function
 * GPIO alternate function mode
 */


/*
 * @GPIO_pu_pd
 * GPIO pull-up or pull-down resistor selection
 */
#define GPIO_PIN_PU         0 // pull-up resistor
#define GPIO_PIN_PD         1 // pull-down resistor
#define GPIO_PIN_OD         2 // open drain
#define GPIO_PIN_NO_PUPD    3


#endif /* REPO_LIB_DRIVERS_GPIO_DRIVER_H_ */
