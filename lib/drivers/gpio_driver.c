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
        if (pGPIOConfig->GPIO_pin_dir == GPIO_OUT) {
            GPIO_PORTA_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTA_DIR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTA_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTA_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTA_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTA_PUR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTA_PDR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTA_ODR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        GPIO_PORTA_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOB) {
        if (pGPIOConfig->GPIO_pin_dir == GPIO_OUT) {
            GPIO_PORTB_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTB_DIR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTB_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTB_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTB_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTB_PUR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTB_PDR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTB_ODR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        GPIO_PORTB_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOC) {
        if (pGPIOConfig->GPIO_pin_dir == GPIO_OUT) {
            GPIO_PORTC_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTC_DIR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTC_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTC_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTC_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTC_PUR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTC_PDR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTC_ODR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        GPIO_PORTC_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOD) {
        if (pGPIOConfig->GPIO_pin_dir == GPIO_OUT) {
            GPIO_PORTD_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTD_DIR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTD_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTD_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTD_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTD_PUR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTD_PDR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTD_ODR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        GPIO_PORTD_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOE) {
        if (pGPIOConfig->GPIO_pin_dir == GPIO_OUT) {
            GPIO_PORTE_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTE_DIR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTE_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTE_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTE_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTE_PUR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTE_PDR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTE_ODR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        GPIO_PORTE_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
    else if (pGPIOConfig->GPIOx == GPIOF) {
        if (pGPIOConfig->GPIO_pin_dir == GPIO_OUT) {
            GPIO_PORTF_DIR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTF_DIR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PU) {
            GPIO_PORTF_PUR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_PD) {
            GPIO_PORTF_PDR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIO_pu_pd == GPIO_PIN_OD) {
            GPIO_PORTF_ODR_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTF_PUR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTF_PDR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
            GPIO_PORTF_ODR_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }

        GPIO_PORTF_DEN_R |= (1 << pGPIOConfig->GPIO_pin_num);
    }
}

uint8_t GPIO_read_pin(GPIO_config_t *pGPIOConfig) {
    uint8_t value;
    if (pGPIOConfig->GPIOx == GPIOA) {
        value = (GPIO_PORTA_DATA_R >> pGPIOConfig->GPIO_pin_num) & 1;
    }
    else if (pGPIOConfig->GPIOx == GPIOB) {
        value = (GPIO_PORTB_DATA_R >> pGPIOConfig->GPIO_pin_num) & 1;
    }
    else if (pGPIOConfig->GPIOx == GPIOC) {
        value = (GPIO_PORTC_DATA_R >> pGPIOConfig->GPIO_pin_num) & 1;
    }
    else if (pGPIOConfig->GPIOx == GPIOD) {
        value = (GPIO_PORTD_DATA_R >> pGPIOConfig->GPIO_pin_num) & 1;
    }
    else if (pGPIOConfig->GPIOx == GPIOE) {
        value = (GPIO_PORTE_DATA_R >> pGPIOConfig->GPIO_pin_num) & 1;
    }
    else if (pGPIOConfig->GPIOx == GPIOF) {
        value = (GPIO_PORTF_DATA_R >> pGPIOConfig->GPIO_pin_num) & 1;
    }
    return value;
}

void GPIO_write_pin(GPIO_config_t *pGPIOConfig, uint8_t high_low) {
    if (high_low == GPIO_PIN_HIGH) {
        if (pGPIOConfig->GPIOx == GPIOA) {
            GPIO_PORTA_DATA_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOB) {
            GPIO_PORTB_DATA_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOC) {
            GPIO_PORTC_DATA_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOD) {
            GPIO_PORTD_DATA_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOE) {
            GPIO_PORTE_DATA_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOF) {
            GPIO_PORTF_DATA_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
    }
    else if (high_low == GPIO_PIN_LOW) {
        if (pGPIOConfig->GPIOx == GPIOA) {
            GPIO_PORTA_DATA_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOB) {
            GPIO_PORTB_DATA_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOC) {
            GPIO_PORTC_DATA_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOD) {
            GPIO_PORTD_DATA_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOE) {
            GPIO_PORTE_DATA_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else if (pGPIOConfig->GPIOx == GPIOF) {
            GPIO_PORTF_DATA_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
    }
}

void GPIO_interrupt_config(GPIO_config_t *pGPIOConfig, uint8_t edge_level_sensitive, uint8_t edge_level_trigger) {
    if (pGPIOConfig->GPIOx == GPIOA) {
        if (edge_level_sensitive == GPIO_EDGE_SENSITIVE) {
            GPIO_PORTA_IS_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTA_IS_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        if (edge_level_trigger == GPIO_LOW_FALLING_TRIGGER) {
            GPIO_PORTA_IEV_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTA_IEV_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTA_ICR_R |= (1 << pGPIOConfig->GPIO_pin_num); // clear any prior interrupt
        GPIO_PORTA_IM_R |= (1 << pGPIOConfig->GPIO_pin_num); // unmask the interrupt
        NVIC_EN0_R |= (1 << 0); // enable interrupts for PORTA
    }
    else if (pGPIOConfig->GPIOx == GPIOB) {
        if (edge_level_sensitive == GPIO_EDGE_SENSITIVE) {
            GPIO_PORTB_IS_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTB_IS_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        if (edge_level_trigger == GPIO_LOW_FALLING_TRIGGER) {
            GPIO_PORTB_IEV_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTB_IEV_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTB_ICR_R |= (1 << pGPIOConfig->GPIO_pin_num); // clear any prior interrupt
        GPIO_PORTB_IM_R |= (1 << pGPIOConfig->GPIO_pin_num); // unmask the interrupt
        NVIC_EN0_R |= (1 << 1); // enable interrupts for PORTB
    }
    else if (pGPIOConfig->GPIOx == GPIOC) {
        if (edge_level_sensitive == GPIO_EDGE_SENSITIVE) {
            GPIO_PORTC_IS_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTC_IS_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        if (edge_level_trigger == GPIO_LOW_FALLING_TRIGGER) {
            GPIO_PORTC_IEV_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTC_IEV_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTC_ICR_R |= (1 << pGPIOConfig->GPIO_pin_num); // clear any prior interrupt
        GPIO_PORTC_IM_R |= (1 << pGPIOConfig->GPIO_pin_num); // unmask the interrupt
        NVIC_EN0_R |= (1 << 2); // enable interrupts for PORTC
    }
    else if (pGPIOConfig->GPIOx == GPIOD) {
        if (edge_level_sensitive == GPIO_EDGE_SENSITIVE) {
            GPIO_PORTD_IS_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTD_IS_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        if (edge_level_trigger == GPIO_LOW_FALLING_TRIGGER) {
            GPIO_PORTD_IEV_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTD_IEV_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTD_ICR_R |= (1 << pGPIOConfig->GPIO_pin_num); // clear any prior interrupt
        GPIO_PORTD_IM_R |= (1 << pGPIOConfig->GPIO_pin_num); // unmask the interrupt
        NVIC_EN0_R |= (1 << 3); // enable interrupts for PORTD
    }
    else if (pGPIOConfig->GPIOx == GPIOE) {
        if (edge_level_sensitive == GPIO_EDGE_SENSITIVE) {
            GPIO_PORTE_IS_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTE_IS_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        if (edge_level_trigger == GPIO_LOW_FALLING_TRIGGER) {
            GPIO_PORTE_IEV_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTE_IEV_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTE_ICR_R |= (1 << pGPIOConfig->GPIO_pin_num); // clear any prior interrupt
        GPIO_PORTE_IM_R |= (1 << pGPIOConfig->GPIO_pin_num); // unmask the interrupt
        NVIC_EN0_R |= (1 << 4); // enable interrupts for PORTE
    }
    else if (pGPIOConfig->GPIOx == GPIOF) {
        NVIC_EN0_R &= ~(1 << 30); // disabling the interrupts on this port before enabling another
        if (edge_level_sensitive == GPIO_EDGE_SENSITIVE) {
            GPIO_PORTF_IS_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTF_IS_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        if (edge_level_trigger == GPIO_LOW_FALLING_TRIGGER) {
            GPIO_PORTF_IEV_R &= ~(1 << pGPIOConfig->GPIO_pin_num);
        }
        else {
            GPIO_PORTF_IEV_R |= (1 << pGPIOConfig->GPIO_pin_num);
        }
        GPIO_PORTF_ICR_R |= (1 << pGPIOConfig->GPIO_pin_num); // clear any prior interrupt
        GPIO_PORTF_IM_R |= (1 << pGPIOConfig->GPIO_pin_num); // unmask the interrupt
        NVIC_EN0_R |= (1 << 30); // enable interrupts for PORTF
    }
}

