/*
 * i2c_driver.h
 *
 *  Created on: Mar. 17, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_DRIVERS_I2C_DRIVER_H_
#define REPO_LIB_DRIVERS_I2C_DRIVER_H_

#include "tm4c123gh6pm.h"
#include <stdint.h>

typedef struct {
    uint8_t I2Cx;
    uint8_t I2C_speed;
}I2C_config_t;

void I2C_pclk_control(uint8_t I2Cx, uint8_t enable_disable);
void I2C_init(I2C_config_t *pI2CConfig);

uint8_t I2C_master_send_data(uint8_t I2Cx, uint8_t slave_addr, uint8_t data);

#define I2C_PCLK_ENABLE     1
#define I2C_PCLK_DISABLE    0

/*
 * @I2Cx
 */
#define I2C0    0
#define I2C1    1
#define I2C2    2
#define I2C3    3

/*
 * @I2C_speed
 */
#define I2C_speed_standard  0x07 // 100Kbps @ 16MHz

#endif /* REPO_LIB_DRIVERS_I2C_DRIVER_H_ */
