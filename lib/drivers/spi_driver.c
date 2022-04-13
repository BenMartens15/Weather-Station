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
            SYSCTL_RCGCSSI_R &= ~(1 << 0); // disable SSI0 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 0); // disable clock to GPIOA for SSI0
        }
        else if(SPIx == SPI1) {
            SYSCTL_RCGCSSI_R &= ~(1 << 1); // disable SSI1 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 5); // disable clock to GPIOF for SSI1
        }
        else if(SPIx == SPI2) {
            SYSCTL_RCGCSSI_R &= ~(1 << 2); // disable SSI2 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 1); // disable clock to GPIOB for SSI2
        }
        else if(SPIx == SPI3) {
            SYSCTL_RCGCSSI_R &= ~(1 << 3); // disable SSI3 clock
            SYSCTL_RCGCGPIO_R &= ~(1 << 3); // disable clock to GPIOD for SSI3
        }
    }
}

void SPI_init(SPI_config_t *pSPIConfig) {
    if (pSPIConfig->SPIx == SPI0) {
        SPI_pclk_control(pSPIConfig->SPIx, SPI_PCLK_ENABLE);

        // configure PORTA2, PORTA5, and PORTA4 for SSI0 clock, Tx, and Rx, respectively
        GPIO_PORTA_AMSEL_R &= ~0x34; // disable analog for these pins
        GPIO_PORTA_DEN_R |= 0x34; // make the pins digital
        GPIO_PORTA_AFSEL_R |= 0x34; // enable alternate function
        GPIO_PORTA_PCTL_R &= ~0x00FF0F00; // assign pins to SSI0
        GPIO_PORTA_PCTL_R |= 0x00220200; // assign pins to SSI0

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
    else if (pSPIConfig->SPIx == SPI1) {
        SPI_pclk_control(pSPIConfig->SPIx, SPI_PCLK_ENABLE);

        // configure PORTF2, PORTF1, and PORTF0 for SSI1 clock, Tx, and Rx, respectively
        GPIO_PORTF_AMSEL_R &= ~0x07; // disable analog for these pins
        GPIO_PORTF_DEN_R |= 0x07; // make the pins digital
        GPIO_PORTF_AFSEL_R |= 0x07; // enable alternate function
        GPIO_PORTF_PCTL_R &= ~0x00000FFF; // assign pins to SSI
        GPIO_PORTF_PCTL_R |= 0x00000222; // assign pins to SSI

        SSI1_CR1_R |= pSPIConfig->SPI_device_mode << 2; // disable SSI and set the SPI mode
        SSI1_CC_R = 0; // use the system clock
        SSI1_CPSR_R = pSPIConfig->SPI_speed;

        uint32_t temp = 0;
        temp |= pSPIConfig->SPI_dss;
        temp |= pSPIConfig->SPI_cpol << 6;
        temp |= pSPIConfig->SPI_cpha << 7;
        SSI1_CR0_R = temp;

        SSI1_CR1_R |= (1 << 1); // enable SPI
    }
    else if (pSPIConfig->SPIx == SPI2) {
        SPI_pclk_control(pSPIConfig->SPIx, SPI_PCLK_ENABLE);

        // configure PORTB4, PORTB7, and PORTB6 for SSI2 clock, Tx, and Rx, respectively
        GPIO_PORTB_AMSEL_R &= ~0xD0; // disable analog for these pins
        GPIO_PORTB_DEN_R |= 0xD0; // make the pins digital
        GPIO_PORTB_AFSEL_R |= 0xD0; // enable alternate function
        GPIO_PORTB_PCTL_R &= ~0xFF0F0000; // assign pins to SSI
        GPIO_PORTB_PCTL_R |= 0x22020000; // assign pins to SSI

        SSI2_CR1_R |= pSPIConfig->SPI_device_mode << 2; // disable SSI and set the SPI mode
        SSI2_CC_R = 0; // use the system clock
        SSI2_CPSR_R = pSPIConfig->SPI_speed;

        uint32_t temp = 0;
        temp |= pSPIConfig->SPI_dss;
        temp |= pSPIConfig->SPI_cpol << 6;
        temp |= pSPIConfig->SPI_cpha << 7;
        SSI2_CR0_R = temp;

        SSI2_CR1_R |= (1 << 1); // enable SPI
    }
    else if (pSPIConfig->SPIx == SPI3) {
        SPI_pclk_control(pSPIConfig->SPIx, SPI_PCLK_ENABLE);

        // configure PORTD0, PORTD3, and PORTD2 for SSI3 clock, Tx, and Rx, respectively
        GPIO_PORTD_AMSEL_R &= ~0x0D; // disable analog for these pins
        GPIO_PORTD_DEN_R |= 0x0D; // make the pins digital
        GPIO_PORTD_AFSEL_R |= 0x0D; // enable alternate function
        GPIO_PORTD_PCTL_R &= ~0x0000FF0F; // assign pins to SSI
        GPIO_PORTD_PCTL_R |= 0x00001101; // assign pins to SSI

        SSI3_CR1_R |= pSPIConfig->SPI_device_mode << 2; // disable SSI and set the SPI mode
        SSI3_CC_R = 0; // use the system clock
        SSI3_CPSR_R = pSPIConfig->SPI_speed;

        uint32_t temp = 0;
        temp |= pSPIConfig->SPI_dss;
        temp |= pSPIConfig->SPI_cpol << 6;
        temp |= pSPIConfig->SPI_cpha << 7;
        SSI3_CR0_R = temp;

        SSI3_CR1_R |= (1 << 1); // enable SPI
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
            GPIO_PORTB_DATA_R &= ~(1 << pin); // assert SS low
        }
        else if (port == SPI_SS_PORT_C) {
            GPIO_PORTC_DATA_R &= ~(1 << pin); // assert SS low
        }
        else if (port == SPI_SS_PORT_D) {
            GPIO_PORTD_DATA_R &= ~(1 << pin); // assert SS low
        }
        else if (port == SPI_SS_PORT_E) {
            GPIO_PORTE_DATA_R &= ~(1 << pin); // assert SS low
        }
        else if (port == SPI_SS_PORT_F) {
            GPIO_PORTF_DATA_R &= ~(1 << pin); // assert SS low
        }
    }
    else {
        if (port == SPI_SS_PORT_A) {
            GPIO_PORTA_DATA_R |= (1 << pin); // assert SS high
        }
        else if (port == SPI_SS_PORT_B) {
            GPIO_PORTB_DATA_R |= (1 << pin); // assert SS high
        }
        else if (port == SPI_SS_PORT_C) {
            GPIO_PORTC_DATA_R |= (1 << pin); // assert SS high
        }
        else if (port == SPI_SS_PORT_D) {
            GPIO_PORTD_DATA_R |= (1 << pin); // assert SS high
        }
        else if (port == SPI_SS_PORT_E) {
            GPIO_PORTE_DATA_R |= (1 << pin); // assert SS high
        }
        else if (port == SPI_SS_PORT_F) {
            GPIO_PORTF_DATA_R |= (1 << pin); // assert SS high
        }
    }
}

void SPI_write_byte(SPI_config_t *pSPIConfig, uint8_t data, uint8_t ss_port, uint8_t ss_pin, uint8_t release) {
    SPI_ss_control(ss_port, ss_pin, SPI_SS_LOW);
    if (pSPIConfig->SPIx == SPI0) {
        while((SSI0_SR_R & 2) == 0); // wait until FIFO is not full
        SSI0_DR_R = data;
        while(SSI0_SR_R & 0x10); // wait until transmit is complete
    }
    else if (pSPIConfig->SPIx == SPI1) {
        while((SSI1_SR_R & 2) == 0); // wait until FIFO is not full
        SSI1_DR_R = data;
        while(SSI1_SR_R & 0x10); // wait until transmit is complete
    }
    else if (pSPIConfig->SPIx == SPI2) {
        while((SSI2_SR_R & 2) == 0); // wait until FIFO is not full
        SSI2_DR_R = data;
        while(SSI2_SR_R & 0x10); // wait until transmit is complete
    }
    else if (pSPIConfig->SPIx == SPI3) {
        while((SSI3_SR_R & 2) == 0); // wait until FIFO is not full
        SSI3_DR_R = data;
        while(SSI3_SR_R & 0x10); // wait until transmit is complete
    }
    if (release) {
        SPI_ss_control(ss_port, ss_pin, SPI_SS_HIGH); // make the slave select pin high again
    }
}

void SPI_write_string(SPI_config_t *pSPIConfig, char* data, uint8_t ss_port, uint8_t ss_pin) {
    SPI_ss_control(ss_port, ss_pin, SPI_SS_LOW);
    if (pSPIConfig->SPIx == SPI0) {
        while((SSI0_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI0_DR_R = data[i];
            while(SSI0_SR_R & 0x10); // wait until transmit is complete
        }
    }
    else if (pSPIConfig->SPIx == SPI1) {
        while((SSI1_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI1_DR_R = data[i];
            while(SSI1_SR_R & 0x10); // wait until transmit is complete
        }
    }
    else if (pSPIConfig->SPIx == SPI2) {
        while((SSI2_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI2_DR_R = data[i];
            while(SSI2_SR_R & 0x10); // wait until transmit is complete
        }
    }
    else if (pSPIConfig->SPIx == SPI3) {
        while((SSI3_SR_R & 2) == 0); // wait until FIFO is not full
        uint8_t i;
        for(i = 0; data[i] != '\0'; i++) {
            SSI3_DR_R = data[i];
            while(SSI3_SR_R & 0x10); // wait until transmit is complete
        }
    }
    SPI_ss_control(ss_port, ss_pin, SPI_SS_HIGH); // make the slave select pin high again
}

void SPI_read_byte(SPI_config_t *pSPIConfig, uint8_t* read_buffer, uint8_t ss_port, uint8_t ss_pin, uint8_t release) {
    SPI_ss_control(ss_port, ss_pin, SPI_SS_LOW);
    if (pSPIConfig->SPIx == SPI0) {
        while(SSI0_SR_R & 3 == 0); // wait until FIFO is not empty
        *read_buffer = SSI0_DR_R;
        while(SSI0_SR_R & 0x10); // wait until receive is complete
    }
    else if (pSPIConfig->SPIx == SPI1) {
        while(SSI1_SR_R & 3 == 0); // wait until FIFO is not empty
        *read_buffer = SSI1_DR_R;
        while(SSI1_SR_R & 0x10); // wait until receive is complete
    }
    else if (pSPIConfig->SPIx == SPI2) {
        while(SSI2_SR_R & 3 == 0); // wait until FIFO is not empty
        *read_buffer = SSI2_DR_R;
        while(SSI2_SR_R & 0x10); // wait until receive is complete
    }
    else if (pSPIConfig->SPIx == SPI3) {
        while(SSI3_SR_R & 3 == 0); // wait until FIFO is not empty
        *read_buffer = SSI3_DR_R;
        while(SSI3_SR_R & 0x10); // wait until receive is complete
    }


    if (release) {
        SPI_ss_control(ss_port, ss_pin, SPI_SS_HIGH); // make the slave select pin high again
    }
}

void SPI_delay(uint16_t time_ms) { // delay for the time specified (in ms)
    uint16_t i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}
