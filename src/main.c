/*
 * main.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "repo/lib/barometric_pressure_sensor/barometric_pressure_sensor.h"

int main() {
    SPI_config_t baro_sensor_spi;
    baro_sensor_spi.SPIx = SPI0;
    baro_sensor_spi.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    baro_sensor_spi.SPI_cpol = SPI_CPOL_LOW;
    baro_sensor_spi.SPI_cpha = SPI_CPHA_LOW;
    baro_sensor_spi.SPI_speed = SPI_SCLK_SPEED_DIV64;
    baro_sensor_spi.SPI_dss = SPI_DSS_8BITS;
}

