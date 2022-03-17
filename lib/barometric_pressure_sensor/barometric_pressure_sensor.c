/*
 * barometric_pressure_sensor.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "barometric_pressure_sensor.h"

void baro_read_coefficients(&pSPIConfig) {
    SPI_write_byte(&SPI_test, 0x88, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x8A, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x8C, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x8E, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x90, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x92, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x94, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x96, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 0x00, SPI_SS_PORT_F, SPI_SS_PIN_2, 1);
}

