/*
 * i2c_test.c
 *
 *  Created on: Mar. 17, 2022
 *      Author: ben
 */

#include "repo/lib/drivers/i2c_driver.h"

#define SLAVE_ADDR 0x68
#define COMMAND_GET_LEN     0x51
#define COMMAND_READ_DATA   0x52

void delay_ms(int time_ms);

int main() {
    uint8_t data_received[32];

    I2C_config_t I2C_test;
    I2C_test.I2Cx = I2C0;
    I2C_test.I2C_speed = I2C_speed_standard;

    I2C_init(&I2C_test);
    delay_ms(1000);
    I2C_master_send_byte(I2C_test.I2Cx, SLAVE_ADDR, 0x52);
    I2C_master_receive_data(I2C_test.I2Cx, SLAVE_ADDR, data_received, 22);

}

void delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}
