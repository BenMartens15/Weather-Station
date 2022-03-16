/*
 * spi_driver.c
 *
 *  Created on: Mar. 15, 2022
 *      Author: ben
 */

#include "spi_driver.h"

void SPI_pclk_control(uint8_t SPIx, uint8_t enable_disable) {
    if (SPIx == SPI0) {
        SYSCTL_RCGCSSI_R |= (1 << 0); // enable SSI0 clock
        SYSCTL_RCGCGPIO_R |= (1 << 0); // enable clock to GPIOA for SSI0
    }
    else if(SPIx == SPI1) {
        SYSCTL_RCGCSSI_R |= (1 << 1); // enable SSI1 clock
        SYSCTL_RCGCGPIO_R |= (1 << 5); // enable clock to GPIOF for SSI1
    }
    else if(SPIx == SPI2) {
        SYSCTL_RCGCSSI_R |= (1 << 2); // enable SSI2 clock
        SYSCTL_RCGCGPIO_R |= (1 << 1); // enable clock to GPIOB for SSI2
    }
    else if(SPIx == SPI3) {
        SYSCTL_RCGCSSI_R |= (1 << 3); // enable SSI3 clock
        SYSCTL_RCGCGPIO_R |= (1 << 3); // enable clock to GPIOD for SSI3
    }
}

void SPI_init(uint8_t SPIx, SPI_config_t *pSPIConfig) {
    if (SPIx == SPI0) {
        // configure PORTA2 and PORTA5 for SSI0 clock and Tx, respectively
        GPIO_PORTA_AMSEL_R &= ~0x24; // disable analog for these pins
        GPIO_PORTA_DEN_R |= 0x24; // make the pins digital
        GPIO_PORTA_AFSEL_R |= 0x24; // enable alternate function
        GPIO_PORTA_PCTL_R &= ~0x00F00F00; // assign pins to SSI0
        GPIO_PORTA_PCTL_R |= 0x00200200; // assign pins to SSI0

        SSI0_CR1_R |= pSPIConfig->SPI_device_mode << 2; // disable SSI and set the SPI mode
        SSI0_CC_R = 0; // use the system clock
        SSI0_CPSR_R = pSPIConfig->SPI_speed;

        uint32_t temp = 0;
        temp |= pSPIConfig->SPI_dss;
        temp |= pSPIConfig->SPI_cpol << 6;
        temp |= pSPIConfig->SPI_cpha << 7;
        SSI0_CR0_R = temp;

        SSI0_CR1_R |= (1 << 1); // enable SPI
    }

}


