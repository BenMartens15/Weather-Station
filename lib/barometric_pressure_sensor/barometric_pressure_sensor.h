/*
 * barometric_pressure_sensor.h
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_BAROMETRIC_PRESSURE_SENSOR_BAROMETRIC_PRESSURE_SENSOR_H_
#define REPO_LIB_BAROMETRIC_PRESSURE_SENSOR_BAROMETRIC_PRESSURE_SENSOR_H_

#include <stdint.h>
#include "repo/lib/drivers/spi_driver.h"

void baro_sensor_init(uint8_t SPIx, uint8_t ss_port, uint8_t ss_pin);
float baro_measure_pressure();

#endif /* REPO_LIB_BAROMETRIC_PRESSURE_SENSOR_BAROMETRIC_PRESSURE_SENSOR_H_ */
