/*
 * i2c_driver.c
 *
 *  Created on: Mar. 17, 2022
 *      Author: ben
 */

#include "i2c_driver.h"

void I2C_pclk_control(uint8_t I2Cx, uint8_t enable_disable) {
    if (enable_disable == I2C_PCLK_ENABLE) {
        if (I2Cx == I2C0) {
            SYSCTL_RCGCI2C_R |= (1 << 0); // enable the clock for I2C0
            SYSCTL_RCGCGPIO_R |= (1 << 1); // enable the clock for GPIOB for I2C0
        }
        else if (I2Cx == I2C1) {
            SYSCTL_RCGCI2C_R |= (1 << 1); // enable the clock for I2C1
            SYSCTL_RCGCGPIO_R |= (1 << 0); // enable the clock for GPIOA for I2C1
        }
        else if (I2Cx == I2C2) {
            SYSCTL_RCGCI2C_R |= (1 << 2); // enable the clock for I2C2
            SYSCTL_RCGCGPIO_R |= (1 << 4); // enable the clock for GPIOE for I2C2
        }
        else if (I2Cx == I2C3) {
            SYSCTL_RCGCI2C_R |= (1 << 3); // enable the clock for I2C3
            SYSCTL_RCGCGPIO_R |= (1 << 3); // enable the clock for GPIOD for I2C3
        }
    }
    else {
        if (I2Cx == I2C0) {
            SYSCTL_RCGCI2C_R &= ~(1 << 0); // enable the clock for I2C0
            SYSCTL_RCGCGPIO_R &= ~(1 << 1); // enable the clock for GPIOB for I2C0
        }
        else if (I2Cx == I2C1) {
            SYSCTL_RCGCI2C_R &= ~(1 << 1); // enable the clock for I2C1
            SYSCTL_RCGCGPIO_R &= ~(1 << 0); // enable the clock for GPIOA for I2C1
        }
        else if (I2Cx == I2C2) {
            SYSCTL_RCGCI2C_R &= ~(1 << 2); // enable the clock for I2C2
            SYSCTL_RCGCGPIO_R &= ~(1 << 4); // enable the clock for GPIOE for I2C2
        }
        else if (I2Cx == I2C3) {
            SYSCTL_RCGCI2C_R &= ~(1 << 3); // enable the clock for I2C3
            SYSCTL_RCGCGPIO_R &= ~(1 << 3); // enable the clock for GPIOD for I2C3
        }
    }
}

void I2C_init(I2C_config_t *pI2CConfig) {
    I2C_pclk_control(pI2CConfig->I2Cx, I2C_PCLK_ENABLE);
    if (pI2CConfig->I2Cx == I2C0) {
        // PB2 = I2C0SCL, PB3 = I2C0SDA
        GPIO_PORTB_AMSEL_R &= ~0x0C; // disable analog for these pins
        GPIO_PORTB_DEN_R |= 0x0C; // make the pins digital
        GPIO_PORTB_AFSEL_R |= 0x0C; // enable alternate function
        GPIO_PORTB_PCTL_R &= ~0x0000FF00;
        GPIO_PORTB_PCTL_R |= 0x00003300; // assign pins to I2C0
        GPIO_PORTB_AHB_ODR_R |= 0x04; // set PORTB3 as open drain
        I2C0_MCR_R = 0x10; // master mode
        I2C0_MTPR_R = pI2CConfig->I2C_speed;
    }
    else if (pI2CConfig->I2Cx == I2C1) {
        // PA6 = I2C1SCL, PA7 = I2C1SDA
        GPIO_PORTA_AMSEL_R &= ~0xC0; // disable analog for these pins
        GPIO_PORTA_DEN_R |= 0xC0; // make the pins digital
        GPIO_PORTA_AFSEL_R |= 0xC0; // enable alternate function
        GPIO_PORTA_PCTL_R &= ~0xFF000000;
        GPIO_PORTA_PCTL_R |= 0x33000000; // assign pins to I2C1
        GPIO_PORTA_AHB_ODR_R |= 0x80; // set PORTA7 as open drain
        I2C1_MCR_R = 0x10; // master mode
        I2C1_MTPR_R = pI2CConfig->I2C_speed;
    }
    else if (pI2CConfig->I2Cx == I2C2) {
        // PE4 = I2C2SCL, PE5 = I2C2SDA
        GPIO_PORTE_AMSEL_R &= ~0x30; // disable analog for these pins
        GPIO_PORTE_DEN_R |= 0x30; // make the pins digital
        GPIO_PORTE_AFSEL_R |= 0x30; // enable alternate function
        GPIO_PORTE_PCTL_R &= ~0x00FF0000;
        GPIO_PORTE_PCTL_R |= 0x00330000; // assign pins to I2C2
        GPIO_PORTE_AHB_ODR_R |= 0x20; // set PORTE5 as open drain
        I2C2_MCR_R = 0x10; // master mode
        I2C2_MTPR_R = pI2CConfig->I2C_speed;
    }
    else if (pI2CConfig->I2Cx == I2C3) {
        // PD0 = I2C3SCL, PD1 = I2C3SDA
        GPIO_PORTD_AMSEL_R &= ~0x03; // disable analog for these pins
        GPIO_PORTD_DEN_R |= 0x03; // make the pins digital
        GPIO_PORTD_AFSEL_R |= 0x03; // enable alternate function
        GPIO_PORTD_PCTL_R &= ~0x000000FF;
        GPIO_PORTD_PCTL_R |= 0x00000033; // assign pins to I2C1
        GPIO_PORTD_AHB_ODR_R |= 0x02; // set PORTD1 as open drain
        I2C3_MCR_R = 0x10; // master mode
        I2C3_MTPR_R = pI2CConfig->I2C_speed;
    }
}

