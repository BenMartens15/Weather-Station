/*
 * lux_sensor.h
 *
 *  Created on: Apr. 7, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_LUX_SENSOR_LUX_SENSOR_H_
#define REPO_LIB_LUX_SENSOR_LUX_SENSOR_H_

#include "repo/lib/drivers/i2c_driver.h"

void lux_sensor_init(uint8_t I2Cx);
uint16_t lux_sensor_read_lux();

#endif /* REPO_LIB_LUX_SENSOR_LUX_SENSOR_H_ */
