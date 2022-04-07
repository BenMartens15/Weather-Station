/*
 * lux_sensor.c
 *
 *  Created on: Apr. 7, 2022
 *      Author: ben
 */

#include "lux_sensor.h"

#define SLAVE_ADDR  0x10

static I2C_config_t lux_sensor;

void lux_sensor_init(uint8_t I2Cx) {
    lux_sensor.I2Cx = I2Cx;
    lux_sensor.I2C_speed = I2C_speed_standard;

    uint8_t write_command[3];

    // ALS_CONF_0 register; ALS shut down = 0, ALS interrupt enable = 0, ALS persistence protect number = 00, ALS integration time = 0000 (100ms), ALS gain = 11 (x1/4)
    write_command[0] = 0;
    write_command[1] = 0x18;
    write_command[2] = 0x00;
    I2C_master_send_data(lux_sensor.I2Cx, SLAVE_ADDR, write_command, 3);

    // ALS_WH register; ALS high threshold = 20000
    write_command[0] = 1;
    write_command[1] = 0x4E;
    write_command[2] = 0x20;
    I2C_master_send_data(lux_sensor.I2Cx, SLAVE_ADDR, write_command, 3);

    // ALS_WL register; ALS low threshold = 10000
    write_command[0] = 2;
    write_command[1] = 0x27;
    write_command[2] = 0x10;
    I2C_master_send_data(lux_sensor.I2Cx, SLAVE_ADDR, write_command, 3);
}

float lux_sensor_read_lux() {
    uint8_t lux_msb_lsb[2];
    float lux;
    I2C_master_send_byte(lux_sensor.I2Cx, SLAVE_ADDR, 4, 0);
    I2C_master_receive_data(lux_sensor.I2Cx, SLAVE_ADDR, lux_msb_lsb, 2);
    lux = (float)((lux_msb_lsb[1] << 8) | lux_msb_lsb[0]) * 0.2304; // 0.0576 is the resolution for integration time of 100ms and ALS gain of 1 (see Application Note for VEML7700)
    return lux;
}

