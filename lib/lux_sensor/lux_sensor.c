/*
 * lux_sensor.c
 *
 *  Created on: Apr. 7, 2022
 *      Author: ben
 */

#include "lux_sensor.h"

#define SLAVE_ADDR  0x10

void lux_sensor_init(uint8_t I2Cx) {
    I2C_config_t lux_sensor;
    lux_sensor.I2Cx = I2Cx;
    lux_sensor.I2C_speed = I2C_speed_standard;

    uint8_t write_command[3];

    // ALS_CONF_0 register; ALS shut down = 0, ALS interrupt enable = 0, ALS persistence protect number = 00, ALS integration time = 0011 (800ms), ALS gain = 00 (x1)
    write_command[0] = 0;
    write_command[1] = 0x00;
    write_command[2] = 0xC0;
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

