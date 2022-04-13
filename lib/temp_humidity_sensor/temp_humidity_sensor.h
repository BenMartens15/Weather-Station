/*
 * temp_humidity_sensor.h
 *
 *  Created on: Mar. 19, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_TEMP_HUMIDITY_SENSOR_TEMP_HUMIDITY_SENSOR_H_
#define REPO_LIB_TEMP_HUMIDITY_SENSOR_TEMP_HUMIDITY_SENSOR_H_

#include "repo/lib/drivers/i2c_driver.h"

// initialize the specified I2C module for communication with the DHT20
void temp_sensor_init(uint8_t I2Cx);

// read the temperature and humidity data from the sensor
void temp_sensor_measure(uint32_t* measurements);

// convert the data to proper format
void temp_sensor_to_decimal(uint32_t* times_ten, uint32_t* integer, uint32_t* decimal);


#endif /* REPO_LIB_TEMP_HUMIDITY_SENSOR_TEMP_HUMIDITY_SENSOR_H_ */
