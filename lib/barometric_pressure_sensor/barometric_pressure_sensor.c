/*
 * barometric_pressure_sensor.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "barometric_pressure_sensor.h"

static void coefficients_to_dec();

static uint8_t baro_ss_port;
static uint8_t baro_ss_pin;
static SPI_config_t baro_sensor;
static uint8_t coefficients_hex[8];

// coefficients used to calculate the barometric pressure
static float a0;
static float b1;
static float b2;
static float c12;

// the procedure for initializing the sensor was taken from the device datasheet
void baro_sensor_init(uint8_t SPIx, uint8_t ss_port, uint8_t ss_pin) {
    baro_sensor.SPIx = SPIx;
    baro_sensor.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    baro_sensor.SPI_cpol = SPI_CPOL_LOW;
    baro_sensor.SPI_cpha = SPI_CPHA_LOW;
    baro_sensor.SPI_speed = SPI_SCLK_SPEED_DIV64;
    baro_sensor.SPI_dss = SPI_DSS_8BITS;

    SPI_init(&baro_sensor);
    SPI_ss_init(ss_port, ss_pin);
    baro_ss_port = ss_port;
    baro_ss_pin = ss_pin;

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
    SPI_write_byte(&baro_sensor, 0x00, ss_port, ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, ss_port, ss_pin, 1);

    coefficients_to_dec();
}

// this function converts the binary coefficients read from the sensor to the proper signed floating point decimal numbers according the datasheet
void coefficients_to_dec() {
    uint16_t binary_coeff;

    // coefficient a0
    binary_coeff = (coefficients_hex[0] << 8) | coefficients_hex[1];
    a0 = binary_coeff >> 3; // the integer bits
    a0 = a0 + (float)((binary_coeff >> 2) & 1) * 1/2
            + (float)((binary_coeff >> 1) & 1) * 1/4
            + (float)((binary_coeff) & 1) * 1/8;

    // coefficient b1
    binary_coeff = (coefficients_hex[2] << 8) | coefficients_hex[3];
    binary_coeff = ~(binary_coeff) + 1; // converting from the 2's complement
    b1 = (binary_coeff >> 13); // the integer bits
    b1 = b1 + (float)((binary_coeff >> 12) & 1) * 1/2
            + (float)((binary_coeff >> 11) & 1) * 1/4
            + (float)((binary_coeff >> 10) & 1) * 1/8
            + (float)((binary_coeff >> 9) & 1) * 1/16
            + (float)((binary_coeff >> 8) & 1) * 1/32
            + (float)((binary_coeff >> 7) & 1) * 1/64
            + (float)((binary_coeff >> 6) & 1) * 1/128
            + (float)((binary_coeff >> 5) & 1) * 1/256
            + (float)((binary_coeff >> 4) & 1) * 1/512
            + (float)((binary_coeff >> 3) & 1) * 1/1024
            + (float)((binary_coeff >> 2) & 1) * 1/2048
            + (float)((binary_coeff >> 1) & 1) * 1/4096
            + (float)((binary_coeff >> 0) & 1) * 1/8192;
    b1 = -1 * b1;

    // coefficient b2
    binary_coeff = (coefficients_hex[4] << 8) | coefficients_hex[5];
    binary_coeff = ~(binary_coeff) + 1; // converting from the 2's complement
    b2 = (binary_coeff >> 14); // the integer bits
    b2 = b2 + (float)((binary_coeff >> 13) & 1) * 1/2
            + (float)((binary_coeff >> 12) & 1) * 1/4
            + (float)((binary_coeff >> 11) & 1) * 1/8
            + (float)((binary_coeff >> 10) & 1) * 1/16
            + (float)((binary_coeff >> 9) & 1) * 1/32
            + (float)((binary_coeff >> 8) & 1) * 1/64
            + (float)((binary_coeff >> 7) & 1) * 1/128
            + (float)((binary_coeff >> 6) & 1) * 1/256
            + (float)((binary_coeff >> 5) & 1) * 1/512
            + (float)((binary_coeff >> 4) & 1) * 1/1024
            + (float)((binary_coeff >> 3) & 1) * 1/2048
            + (float)((binary_coeff >> 2) & 1) * 1/4096
            + (float)((binary_coeff >> 1) & 1) * 1/8192
            + (float)((binary_coeff >> 0) & 1) * 1/16384;
    b2 = -1 * b2;

    // coefficient c12
    binary_coeff = (coefficients_hex[6] << 8) | coefficients_hex[7];
    binary_coeff = binary_coeff >> 2; // getting rid of the zeros that are not used in this coefficient
    c12 = (float)((binary_coeff >> 11) & 1) * 1/2048
            + (float)((binary_coeff >> 10) & 1) * 1/4096
            + (float)((binary_coeff >> 9) & 1) * 1/8192
            + (float)((binary_coeff >> 8) & 1) * 1/16384
            + (float)((binary_coeff >> 7) & 1) * 1/32768
            + (float)((binary_coeff >> 6) & 1) * 1/65536
            + (float)((binary_coeff >> 5) & 1) * 1/131072
            + (float)((binary_coeff >> 4) & 1) * 1/262144
            + (float)((binary_coeff >> 3) & 1) * 1/524288;
}

float baro_measure_pressure() {
    uint16_t Padc;
    uint16_t Tadc;
    uint8_t pressure_temperature[4];

    // start temperature and pressure conversions
    uint8_t clear_read;
    SPI_write_byte(&baro_sensor, 0x24, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 1);
    SPI_delay(10);

    // read the pressure and temperature ADC counts
    SPI_write_byte(&baro_sensor, 0x80, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &pressure_temperature[0], baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x82, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &pressure_temperature[1], baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x84, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &pressure_temperature[2], baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x86, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &pressure_temperature[3], baro_ss_port, baro_ss_pin, 0);
    SPI_write_byte(&baro_sensor, 0x00, baro_ss_port, baro_ss_pin, 0);
    SPI_read_byte(&baro_sensor, &clear_read, baro_ss_port, baro_ss_pin, 1);

    Padc = ((pressure_temperature[0] << 8) | pressure_temperature[1]) >> 6;
    Tadc = ((pressure_temperature[2] << 8) | pressure_temperature[3]) >> 6;

    float c12x2;
    float a1;
    float a1x1;
    float y1;
    float a2x2;
    float Pcomp;

    // calculate the pressure in kPa
    c12x2 = c12 * Tadc;
    a1 = b1 + c12x2;
    a1x1 = a1 * Padc;
    y1 = a0 + a1x1;
    a2x2 = b2 * Tadc;
    Pcomp = y1 + a2x2;

    return ((65.0 / 1023.0) * Pcomp) + 50;
}

