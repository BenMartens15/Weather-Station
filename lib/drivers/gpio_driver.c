/*
 * gpio_driver.c
 *
 *  Created on: Mar. 21, 2022
 *      Author: ben
 */

#include "gpio_driver.h"

void GPIO_pclk_control(uint8_t GPIOx, uint8_t enable_disable) {
    if (GPIOx == GPIOA) {
        SYSCTL_RCGCGPIO_R |= (1 << 0); // enable the clock for GPIOA
    }
    else if (GPIOx == GPIOB) {
        SYSCTL_RCGCGPIO_R |= (1 << 1); // enable the clock for GPIOB
    }
    else if (GPIOx == GPIOC) {
        SYSCTL_RCGCGPIO_R |= (1 << 2); // enable the clock for GPIOC
    }
    else if (GPIOx == GPIOD) {
        SYSCTL_RCGCGPIO_R |= (1 << 3); // enable the clock for GPIOD
    }
    else if (GPIOx == GPIOE) {
        SYSCTL_RCGCGPIO_R |= (1 << 4); // enable the clock for GPIOE
    }
    else if (GPIOx == GPIOF) {
        SYSCTL_RCGCGPIO_R |= (1 << 5); // enable the clock for GPIOF
    }
}

void GPIO_init(GPIO_config_t *pGPIOConfig) {
    GPIO_pclk_control(pGPIOConfig->GPIOx, GPIO_PCLK_ENABLE);
    if (pGPIOConfig->GPIOx == GPIOA) {
        GPIO_PORTA_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTA_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTA_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTA_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTA_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOB) {
        GPIO_PORTB_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTB_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTB_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTB_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTB_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOC) {
        GPIO_PORTC_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTC_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTC_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTC_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTC_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOD) {
        GPIO_PORTD_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTD_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTD_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTD_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTD_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOE) {
        GPIO_PORTE_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTE_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTE_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTE_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTE_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOF) {
        GPIO_PORTF_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTF_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTF_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTF_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTF_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
}
