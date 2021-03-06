/*
 * i2c_driver.c
 *
 *  Created on: Mar. 17, 2022
 *      Author: ben
 */

#include "i2c_driver.h"

static int I2C_wait_busy(uint8_t I2Cx) {
    if (I2Cx == I2C0) {
        while(I2C0_MCS_R & 1); // wait until master is not busy
        return I2C0_MCS_R & 0xE;
    }
    else if (I2Cx == I2C1) {
        while(I2C1_MCS_R & 1);
        return I2C1_MCS_R & 0xE;
    }
    else if (I2Cx == I2C2) {
        while(I2C2_MCS_R & 1);
        return I2C2_MCS_R & 0xE;
    }
    else {
        while(I2C3_MCS_R & 1);
        return I2C3_MCS_R & 0xE;
    }
}

void I2C_pclk_control(uint8_t I2Cx, uint8_t enable_disable) {
    if (enable_disable == I2C_PCLK_ENABLE) {
        if (I2Cx == I2C0) {
            SYSCTL_RCGCI2C_R |= (1 << 0); // enable the clock for I2C0
            SYSCTL_RCGCGPIO_R |= (1 << 1); // enable the clock for GPIOB for I2C0
        }
        else if (I2Cx == I2C1) {
            SYSCTL_RCGCI2C_R |= (1 << 1); // enable the clock for I2C3
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
        else if (I2Cx == I2C3) {
            SYSCTL_RCGCI2C_R &= ~(1 << 1); // enable the clock for I2C3
            SYSCTL_RCGCGPIO_R &= ~(1 << 0); // enable the clock for GPIOA for I2C3
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
        GPIO_PORTB_AFSEL_R |= 0x0C; // enable alternate function
        GPIO_PORTB_PCTL_R &= ~0x0000FF00;
        GPIO_PORTB_PCTL_R |= 0x00003300; // assign pins to I2C0
        GPIO_PORTB_DEN_R |= 0x0C; // make the pins digital
        GPIO_PORTB_ODR_R |= 0x08; // set PORTB3 as open drain
        I2C0_MCR_R = 0x10; // master mode
        I2C0_MTPR_R = pI2CConfig->I2C_speed;
    }
    else if (pI2CConfig->I2Cx == I2C1) {
        // PA6 = I2C1SCL, PA7 = I2C1SDA
        GPIO_PORTA_AFSEL_R |= 0xC0; // enable alternate function
        GPIO_PORTA_PCTL_R &= ~0xFF000000;
        GPIO_PORTA_PCTL_R |= 0x33000000; // assign pins to I2C3
        GPIO_PORTA_DEN_R |= 0xC0; // make the pins digital
        GPIO_PORTA_ODR_R |= 0x80; // set PORTA7 as open drain
        I2C1_MCR_R = 0x10; // master mode
        I2C1_MTPR_R = pI2CConfig->I2C_speed;
    }
    else if (pI2CConfig->I2Cx == I2C2) {
        // PE4 = I2C2SCL, PE5 = I2C2SDA
        GPIO_PORTE_AFSEL_R |= 0x30; // enable alternate function
        GPIO_PORTE_PCTL_R &= ~0x00FF0000;
        GPIO_PORTE_PCTL_R |= 0x00330000; // assign pins to I2C2
        GPIO_PORTE_DEN_R |= 0x30; // make the pins digital
        GPIO_PORTE_ODR_R |= 0x20; // set PORTE5 as open drain
        I2C2_MCR_R = 0x10; // master mode
        I2C2_MTPR_R = pI2CConfig->I2C_speed;
    }
    else if (pI2CConfig->I2Cx == I2C3) {
        // PD0 = I2C3SCL, PD1 = I2C3SDA
        GPIO_PORTD_AFSEL_R |= 0x03; // enable alternate function
        GPIO_PORTD_PCTL_R &= ~0x000000FF;
        GPIO_PORTD_PCTL_R |= 0x00000033; // assign pins to I2C3
        GPIO_PORTD_DEN_R |= 0x03; // make the pins digital
        GPIO_PORTD_ODR_R |= 0x02; // set PORTD1 as open drain
        I2C3_MCR_R = 0x10; // master mode
        I2C3_MTPR_R = pI2CConfig->I2C_speed;
    }
}

uint8_t I2C_master_send_byte(uint8_t I2Cx, uint8_t slave_addr, uint8_t data, uint8_t generate_stop) {
    uint8_t error;
    if (I2Cx == I2C0) {
        I2C0_MSA_R = slave_addr << 1;
        I2C0_MDR_R = data;
        while(I2C0_MCS_R & 0x80); // wait until the I2C bus is not busy
        if (generate_stop) {
            I2C0_MCS_R = 0x07; // initiate a single byte transmit of data followed by a stop condition
        }
        else {
            I2C0_MCS_R = 0x03; // initiate a single byte transmit of data with no stop condition
        }
        error = I2C_wait_busy(I2Cx);
        if(error) {
            return error;
        }
        return 0;
    }
    else if (I2Cx == I2C1) {
        I2C1_MSA_R = slave_addr << 1;
        I2C1_MDR_R = data;
        while(I2C1_MCS_R & 0x80); // wait until the I2C bus is not busy
        if (generate_stop) {
            I2C1_MCS_R = 0x07; // initiate a single byte transmit of data followed by a stop condition
        }
        else {
            I2C1_MCS_R = 0x03; // initiate a single byte transmit of data with no stop condition
        }
        error = I2C_wait_busy(I2Cx);
        if(error) {
            return error;
        }
        return 0;
    }
    else if (I2Cx == I2C2) {
        I2C2_MSA_R = slave_addr << 1;
        I2C2_MDR_R = data;
        while(I2C2_MCS_R & 0x80); // wait until the I2C bus is not busy
        if (generate_stop) {
            I2C2_MCS_R = 0x07; // initiate a single byte transmit of data followed by a stop condition
        }
        else {
            I2C2_MCS_R = 0x03; // initiate a single byte transmit of data with no stop condition
        }
        error = I2C_wait_busy(I2Cx);
        if(error) {
            return error;
        }
        return 0;
    }
    else if (I2Cx == I2C3) {
        I2C3_MSA_R = slave_addr << 1;
        I2C3_MDR_R = data;
        while(I2C3_MCS_R & 0x80); // wait until the I2C bus is not busy
        if (generate_stop) {
            I2C3_MCS_R = 0x07; // initiate a single byte transmit of data followed by a stop condition
        }
        else {
            I2C3_MCS_R = 0x03; // initiate a single byte transmit of data with no stop condition
        }
        error = I2C_wait_busy(I2Cx);
        if(error) {
            return error;
        }
        return 0;
    }
    return 0;
}

uint8_t I2C_master_receive_byte(uint8_t I2Cx, uint8_t slave_addr, uint8_t* receive_buffer) {
    uint8_t error;
    if (I2Cx == I2C0) {
        I2C0_MSA_R = 0; // resetting the address register
        I2C0_MSA_R |= (1 << 0); // set the R/S bit to 1 (receive)
        I2C0_MSA_R |= slave_addr << 1;
        while(I2C0_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C0_MCS_R = 0x07; // initiate a single byte transmit of data
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        *receive_buffer = I2C0_MDR_R;
        return 0;
    }
    else if (I2Cx == I2C1) {
        I2C1_MSA_R = 0; // resetting the address register
        I2C1_MSA_R |= (1 << 0); // set the R/S bit to 1 (receive)
        I2C1_MSA_R |= slave_addr << 1;
        while(I2C1_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C1_MCS_R = 0x07; // initiate a single byte transmit of data
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        *receive_buffer = I2C1_MDR_R;
        return 0;
    }
    else if (I2Cx == I2C2) {
        I2C2_MSA_R = 0; // resetting the address register
        I2C2_MSA_R |= (1 << 0); // set the R/S bit to 1 (receive)
        I2C2_MSA_R |= slave_addr << 1;
        while(I2C2_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C2_MCS_R = 0x07; // initiate a single byte transmit of data
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        *receive_buffer = I2C2_MDR_R;
        return 0;
    }
    else if (I2Cx == I2C3) {
        I2C3_MSA_R = 0; // resetting the address register
        I2C3_MSA_R |= (1 << 0); // set the R/S bit to 1 (receive)
        I2C3_MSA_R |= slave_addr << 1;
        while(I2C3_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C3_MCS_R = 0x07; // initiate a single byte transmit of data
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        *receive_buffer = I2C3_MDR_R;
        return 0;
    }
    return 0;
}

uint8_t I2C_master_send_data(uint8_t I2Cx, uint8_t slave_addr, uint8_t* data, uint8_t num_bytes) {
    uint8_t error;
    if (I2Cx == I2C0) {
        I2C0_MSA_R = 0; // resetting the address register
        I2C0_MSA_R = slave_addr << 1;
        I2C0_MDR_R = data[0];
        while(I2C0_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C0_MCS_R = 0x03; // generate start condition
        uint8_t reg = I2C0_MCS_R;
        uint8_t index;
        for (index = 1; index < num_bytes; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C0_MCS_R & 5) { // if arbitration was lost
                    I2C0_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            I2C0_MDR_R = data[index];
            I2C0_MCS_R = 0x01;
        }
        I2C0_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        return 0;
    }
    else if (I2Cx == I2C1) {
        I2C1_MSA_R = 0; // resetting the address register
        I2C1_MSA_R |= slave_addr << 1;
        I2C1_MDR_R = data[0];
        while(I2C1_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C1_MCS_R = 0x03; // generate start condition
        uint8_t reg = I2C1_MCS_R;
        uint8_t index;
        for (index = 1; index < num_bytes; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C1_MCS_R & 5) { // if arbitration was lost
                    I2C1_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            I2C1_MDR_R = data[index];
            I2C1_MCS_R = 0x01;
        }
        I2C1_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        return 0;
    }
    else if (I2Cx == I2C2) {
        I2C2_MSA_R = 0; // resetting the address register
        I2C2_MSA_R |= slave_addr << 1;
        I2C2_MDR_R = data[0];
        while(I2C2_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C2_MCS_R = 0x03; // generate start condition
        uint8_t reg = I2C2_MCS_R;
        uint8_t index;
        for (index = 1; index < num_bytes; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C2_MCS_R & 5) { // if arbitration was lost
                    I2C2_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            I2C2_MDR_R = data[index];
            I2C2_MCS_R = 0x01;
        }
        I2C2_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        return 0;
    }
    else if (I2Cx == I2C3) {
        I2C3_MSA_R = 0; // resetting the address register
        I2C3_MSA_R |= slave_addr << 1;
        I2C3_MDR_R = data[0];
        while(I2C3_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C3_MCS_R = 0x03; // generate start condition
        uint8_t reg = I2C3_MCS_R;
        uint8_t index;
        for (index = 1; index < num_bytes; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C3_MCS_R & 5) { // if arbitration was lost
                    I2C3_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            I2C3_MDR_R = data[index];
            I2C3_MCS_R = 0x01;
        }
        I2C3_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        return 0;
    }
    return 0;
}

uint8_t I2C_master_receive_data(uint8_t I2Cx, uint8_t slave_addr, uint8_t* receive_buffer, uint8_t num_bytes) {
    uint8_t error;
    if (I2Cx == I2C0) {
        I2C0_MSA_R |= (1 << 0); // setting R/S bit to 1 for reading
        I2C0_MSA_R |= slave_addr << 1;
        while(I2C0_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C0_MCS_R = 0x0B; // generate start condition and enable automatic ACKing
        uint8_t index;
        for (index = 0; index < num_bytes - 1; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C0_MCS_R & 5) { // if arbitration was lost
                    I2C0_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            receive_buffer[index] = I2C0_MDR_R;
            I2C0_MCS_R = 0x09;
        }
        I2C0_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        receive_buffer[num_bytes - 1] = I2C0_MDR_R;
    }
    else if (I2Cx == I2C1) {
        I2C1_MSA_R |= (1 << 0); // setting R/S bit to 1 for reading
        I2C1_MSA_R |= slave_addr << 1;
        while(I2C1_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C1_MCS_R = 0x0B; // generate start condition and enable automatic ACKing
        uint8_t index;
        for (index = 0; index < num_bytes - 1; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C1_MCS_R & 5) { // if arbitration was lost
                    I2C1_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            receive_buffer[index] = I2C1_MDR_R;
            I2C1_MCS_R = 0x09;
        }
        I2C1_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        receive_buffer[num_bytes - 1] = I2C1_MDR_R;
    }
    else if (I2Cx == I2C2) {
        I2C2_MSA_R |= (1 << 0); // setting R/S bit to 1 for reading
        I2C2_MSA_R |= slave_addr << 1;
        while(I2C2_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C2_MCS_R = 0x0B; // generate start condition and enable automatic ACKing
        uint8_t index;
        for (index = 0; index < num_bytes - 1; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C2_MCS_R & 5) { // if arbitration was lost
                    I2C2_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            receive_buffer[index] = I2C2_MDR_R;
            I2C2_MCS_R = 0x09;
        }
        I2C2_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        receive_buffer[num_bytes - 1] = I2C2_MDR_R;
    }
    else if (I2Cx == I2C3) {
        I2C3_MSA_R |= (1 << 0); // setting R/S bit to 1 for reading
        I2C3_MSA_R |= slave_addr << 1;
        while(I2C3_MCS_R & 0x80); // wait until the I2C bus is not busy
        I2C3_MCS_R = 0x0B; // generate start condition and enable automatic ACKing
        uint8_t index;
        for (index = 0; index < num_bytes - 1; index++) {
            error = I2C_wait_busy(I2Cx);
            if (error) {
                if (I2C3_MCS_R & 5) { // if arbitration was lost
                    I2C3_MCS_R = 0x04; // generate stop condition
                }
                return error;
            }
            receive_buffer[index] = I2C3_MDR_R;
            I2C3_MCS_R = 0x09;
        }
        I2C3_MCS_R = 0x05; // generate stop condition
        error = I2C_wait_busy(I2Cx);
        if (error) {
            return error;
        }
        receive_buffer[num_bytes - 1] = I2C3_MDR_R;
    }
    return 0;
}
