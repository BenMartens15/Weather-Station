/*
 * barometric_pressure_sensor.h
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_BAROMETRIC_PRESSURE_SENSOR_BAROMETRIC_PRESSURE_SENSOR_H_
#define REPO_LIB_BAROMETRIC_PRESSURE_SENSOR_BAROMETRIC_PRESSURE_SENSOR_H_

#include "repo/lib/drivers/spi_driver.h"

void baro_read_coefficients(SPI_config_t *pSPIConfig);

#endif /* REPO_LIB_BAROMETRIC_PRESSURE_SENSOR_BAROMETRIC_PRESSURE_SENSOR_H_ */
