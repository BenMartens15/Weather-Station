/*
 * gpio_driver.h
 *
 *  Created on: Mar. 21, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_DRIVERS_GPIO_DRIVER_H_
#define REPO_LIB_DRIVERS_GPIO_DRIVER_H_

#include "repo/lib/drivers/tm4c123gh6pm.h"

typedef struct {
    uint8_t GPIOx; // GPIO port
    uint8_t GPIO_pin_num;
    uint8_t GPIO_pin_dir; // pin direction
    uint8_t GPIO_alt_function; // alternate function mode
    uint8_t GPIO_pu_pd; // pull-up/pull-down resistor control
}GPIO_config_t;

void GPIO_pclk_control(uint8_t GPIOx, uint8_t enable_disable);
void GPIO_init(GPIO_config_t *pGPIOConfig);

#define GPIO_PCLK_ENABLE    1
#define GPIO_PCLK_DISABLE   0

/*
 * @GPIOx
 */
#define GPIOA               0
#define GPIOB               1
#define GPIOC               2
#define GPIOD               3
#define GPIOE               4
#define GPIOF               5

/*
 * @GPIO_pin_num
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
 */
#define GPIO_IN             0
#define GPIO_OUT            1
#define GPIO_ALTFN          2

/*
 * @GPIO_alt_function
 */


/*
 * @GPIO_pu_pd
 */
#define GPIO_PIN_PU         0 // pull-up resistor
#define GPIO_PIN_PD         1 // pull-down resistor
#define GPIO_PIN_OD         2 // open drain


#endif /* REPO_LIB_DRIVERS_GPIO_DRIVER_H_ */
