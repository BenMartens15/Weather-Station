/*
 * spi_driver.c
 *
 *  Created on: Mar. 15, 2022
 *      Author: ben
 */

#include "spi_driver.h"

void SPI_ss_control(uint8_t port, uint8_t pin, uint8_t high_low);

void SPI_pclk_control(uint8_t SPIx, uint8_t enable_disable) {
    if (enable_disable == SPI_PCLK_ENABLE) {
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
    else {
        if (SPIx == SPI0) {
            SYSCTL_RCGCSSI_R &= ~(1 << 0); // enable SSI0 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 0); // enable clock to GPIOA for SSI0
        }
        else if(SPIx == SPI1) {
            SYSCTL_RCGCSSI_R &= ~(1 << 1); // enable SSI1 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 5); // enable clock to GPIOF for SSI1
        }
        else if(SPIx == SPI2) {
            SYSCTL_RCGCSSI_R &= ~(1 << 2); // enable SSI2 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 1); // enable clock to GPIOB for SSI2
        }
        else if(SPIx == SPI3) {
            SYSCTL_RCGCSSI_R &= ~(1 << 3); // enable SSI3 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 3); // enable clock to GPIOD for SSI3
        }
    }
}

void SPI_init(uint8_t SPIx, SPI_config_t *pSPIConfig) {
    if (SPIx == SPI0) {
        SPI_pclk_control(SPIx, SPI_PCLK_ENABLE);

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

void SPI_ss_init(uint8_t port, uint8_t pin) {
    SYSCTL_RCGCGPIO_R |= (1 << port); // enable clock to the slave select GPIO port
    if (port == SPI_SS_PORT_A) {
        GPIO_PORTA_DEN_R |= (1 << pin); // make the pin digital
        GPIO_PORTA_DIR_R |= (1 << pin); // make the pin output
        GPIO_PORTA_DATA_R |= (1 << pin); // keep SS idle high
    }
    else if (port == SPI_SS_PORT_B) {
        GPIO_PORTB_DEN_R |= (1 << pin); // make the pin digital
        GPIO_PORTB_DIR_R |= (1 << pin); // make the pin output
        GPIO_PORTB_DATA_R |= (1 << pin); // keep SS idle high
    }
    else if (port == SPI_SS_PORT_C) {
        GPIO_PORTC_DEN_R |= (1 << pin); // make the pin digital
        GPIO_PORTC_DIR_R |= (1 << pin); // make the pin output
        GPIO_PORTC_DATA_R |= (1 << pin); // keep SS idle high
    }
    else if (port == SPI_SS_PORT_D) {
        GPIO_PORTD_DEN_R |= (1 << pin); // make the pin digital
        GPIO_PORTD_DIR_R |= (1 << pin); // make the pin output
        GPIO_PORTD_DATA_R |= (1 << pin); // keep SS idle high
    }
    else if (port == SPI_SS_PORT_E) {
        GPIO_PORTE_DEN_R |= (1 << pin); // make the pin digital
        GPIO_PORTE_DIR_R |= (1 << pin); // make the pin output
        GPIO_PORTE_DATA_R |= (1 << pin); // keep SS idle high
    }
    else if (port == SPI_SS_PORT_F) {
        GPIO_PORTF_DEN_R |= (1 << pin); // make the pin digital
        GPIO_PORTF_DIR_R |= (1 << pin); // make the pin output
        GPIO_PORTF_DATA_R |= (1 << pin); // keep SS idle high
    }
}

void SPI_ss_control(uint8_t port, uint8_t pin, uint8_t high_low) {
    if(high_low == SPI_SS_LOW) {
        if (port == SPI_SS_PORT_A) {
            GPIO_PORTA_DATA_R &= ~(1 << pin); // assert SS low
        }
        else if (port == SPI_SS_PORT_B) {
            GPIO_PORTB_DATA_R &= ~(1 << pin);
        }
        else if (port == SPI_SS_PORT_C) {
            GPIO_PORTC_DATA_R &= ~(1 << pin);
        }
        else if (port == SPI_SS_PORT_D) {
            GPIO_PORTD_DATA_R &= ~(1 << pin);
        }
        else if (port == SPI_SS_PORT_E) {
            GPIO_PORTE_DATA_R &= ~(1 << pin);
        }
        else if (port == SPI_SS_PORT_F) {
            GPIO_PORTF_DATA_R &= ~(1 << pin);
        }
    }
    else {
        if (port == SPI_SS_PORT_A) {
            GPIO_PORTA_DATA_R |= (1 << pin); // assert SS high
        }
        else if (port == SPI_SS_PORT_B) {
            GPIO_PORTB_DATA_R |= (1 << pin);
        }
        else if (port == SPI_SS_PORT_C) {
            GPIO_PORTC_DATA_R |= (1 << pin);
        }
        else if (port == SPI_SS_PORT_D) {
            GPIO_PORTD_DATA_R |= (1 << pin);
        }
        else if (port == SPI_SS_PORT_E) {
            GPIO_PORTE_DATA_R |= (1 << pin);
        }
        else if (port == SPI_SS_PORT_F) {
            GPIO_PORTF_DATA_R |= (1 << pin);
        }
    }
}

void SPI_write_char(uint8_t SPIx, SPI_config_t *pSPIConfig, char data, uint8_t ss_port, uint8_t ss_pin) {
    SPI_ss_control(ss_port, ss_pin, SPI_SS_LOW);
    if (SPIx == SPI0) {
        while((SSI0_SR_R & 2) == 0); // wait until FIFO is not full
        SSI0_DR_R = data;
        while(SSI0_SR_R & 0x10); // wait until transmit is complete
    }
    else if (SPIx == SPI1) {
        while((SSI1_SR_R & 2) == 0); // wait until FIFO is not full
        SSI1_DR_R = data;
        while(SSI1_SR_R & 0x10); // wait until transmit is complete
    }
    else if (SPIx == SPI2) {
        while((SSI2_SR_R & 2) == 0); // wait until FIFO is not full
        SSI2_DR_R = data;
        while(SSI2_SR_R & 0x10); // wait until transmit is complete
    }
    else if (SPIx == SPI3) {
        while((SSI3_SR_R & 2) == 0); // wait until FIFO is not full
        SSI3_DR_R = data;
        while(SSI3_SR_R & 0x10); // wait until transmit is complete
    }
    SPI_ss_control(ss_port, ss_pin, SPI_SS_HIGH);
}

void SPI_write_string(uint8_t SPIx, SPI_config_t *pSPIConfig, char* data, uint8_t ss_port, uint8_t ss_pin) {
    SPI_ss_control(ss_port, ss_pin, SPI_SS_LOW);
    if (SPIx == SPI0) {
        while((SSI0_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI0_DR_R = data[i];
            while(SSI0_SR_R & 0x10); // wait until transmit is complete
        }
    }
    else if (SPIx == SPI1) {
        while((SSI1_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI1_DR_R = data[i];
            while(SSI1_SR_R & 0x10); // wait until transmit is complete
        }
    }
    else if (SPIx == SPI2) {
        while((SSI2_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI2_DR_R = data[i];
            while(SSI2_SR_R & 0x10); // wait until transmit is complete
        }
    }
    else if (SPIx == SPI3) {
        while((SSI3_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI3_DR_R = data[i];
            while(SSI3_SR_R & 0x10); // wait until transmit is complete
        }
    }
    SPI_ss_control(ss_port, ss_pin, SPI_SS_HIGH);
}


