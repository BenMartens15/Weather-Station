/*
 * barometric_pressure_sensor.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "barometric_pressure_sensor.h"

static void coefficients_to_dec();

static SPI_config_t baro_sensor;
static uint8_t coefficients_hex[8];
static float coefficients_dec[8];

void baro_sensor_init(uint8_t SPIx, uint8_t ss_port, uint8_t ss_pin) {
    baro_sensor.SPIx = SPIx;
    baro_sensor.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    baro_sensor.SPI_cpol = SPI_CPOL_LOW;
    baro_sensor.SPI_cpha = SPI_CPHA_LOW;
    baro_sensor.SPI_speed = SPI_SCLK_SPEED_DIV64;
    baro_sensor.SPI_dss = SPI_DSS_8BITS;

    SPI_init(&baro_sensor);
    SPI_ss_init(ss_port, ss_pin);

    // read the device coefficients_hex (used in the pressure calculation, are device specific and never change)
    uint8_t clear_read;

    SPI_ss_control(ss_port, ss_pin, SPI_SS_LOW);

    SPI_write_byte(&baro_sensor, 0x88, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[0], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x8A, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[1], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x8C, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[2], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x8E, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[3], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x90, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[4], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x92, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[5], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x94, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[6], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x96, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &coefficients_hex[7], ss_port, ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 1);

    coefficients_to_dec();
}

void coefficients_to_dec() {
    uint16_t binary_coeff;
    binary_coeff = (coefficients_hex[0] << 8) | coefficients_hex[1];
    coefficients_dec[0] = binary_coeff >> 3;
    coefficients_dec[0] = coefficients_dec[0] + (float)((binary_coeff >> 2) & 1) * 1/2 + (float)((binary_coeff >> 1) & 1) * 1/4 + (float)((binary_coeff) & 1) * 1/8;
    if ((binary_coeff >> 15) & 1) { // checking the sign of the number
        coefficients_dec[0] = -1 * coefficients_dec[0];
    }
    SPI_delay(1000);
}

