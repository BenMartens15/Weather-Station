/*
 * temp_humidity_sensor.h
 *
 *  Created on: Mar. 19, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_TEMP_HUMIDITY_SENSOR_TEMP_HUMIDITY_SENSOR_H_
#define REPO_LIB_TEMP_HUMIDITY_SENSOR_TEMP_HUMIDITY_SENSOR_H_

#include "repo/lib/drivers/i2c_driver.h"

void temp_sensor_init(uint8_t I2Cx);
void temp_sensor_measure(uint32_t* measurements);


#endif /* REPO_LIB_TEMP_HUMIDITY_SENSOR_TEMP_HUMIDITY_SENSOR_H_ */
