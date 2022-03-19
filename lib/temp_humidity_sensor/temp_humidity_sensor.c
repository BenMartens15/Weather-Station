/*
 * temp_humidity_sensor.c
 *
 *  Created on: Mar. 19, 2022
 *      Author: ben
 */

#include "temp_humidity_sensor.h"

void delay_ms(int time_ms) {
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

/*
 * This function follows the initialization process from the function "AHT20_Start_Init"
 * in the downloadable sample program found at http://www.aosong.com/en/products-67.html
 */
void temp_sensor_init(uint8_t I2Cx) {
    uint8_t register_clear[] = {0x1B, 0x00, 0x00};
    uint8_t register_init[3];
    uint8_t received_bytes[3];

    I2C_config_t temp_sensor;
    temp_sensor.I2Cx = I2Cx;
    temp_sensor.I2C_speed = I2C_speed_standard;

    I2C_init(&temp_sensor);

    // initializing register 0x1B
    I2C_master_send_data(I2Cx, 0x70, register_clear, 3);
    delay_ms(5);
    I2C_master_receive_data(I2Cx, 0x70, received_bytes, 3);
    delay_ms(10);
    register_init[0] = 0xB0 | 0x1B; // register command
    register_init[1] = received_bytes[1];
    register_init[2] = received_bytes[2];
    I2C_master_send_data(I2Cx, 0x70, register_init, 3);

    // initializing register 0x1C
    register_clear[0] = 0x1C;
    I2C_master_send_data(I2Cx, 0x70, register_clear, 3);
    delay_ms(5);
    I2C_master_receive_data(I2Cx, 0x70, received_bytes, 3);
    delay_ms(10);
    register_init[0] = 0xB0 | 0x1C; // register command
    register_init[1] = received_bytes[1];
    register_init[2] = received_bytes[2];
    I2C_master_send_data(I2Cx, 0x70, register_init, 3);

    // initializing register 0x1E
    register_clear[0] = 0x1E;
    I2C_master_send_data(I2Cx, 0x70, register_clear, 3);
    delay_ms(5);
    I2C_master_receive_data(I2Cx, 0x70, received_bytes, 3);
    delay_ms(10);
    register_init[0] = 0xB0 | 0x1E; // register command
    register_init[1] = received_bytes[1];
    register_init[2] = received_bytes[2];
    I2C_master_send_data(I2Cx, 0x70, register_init, 3);
}
