/*
 * temp_humidity_sensor.c
 *
 *  Created on: Mar. 19, 2022
 *      Author: ben
 */

#include "temp_humidity_sensor.h"

#define SLAVE_ADDRESS 0x38

static I2C_config_t temp_sensor;

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
    uint8_t sensor_status;

    temp_sensor.I2Cx = I2Cx;
    temp_sensor.I2C_speed = I2C_speed_standard;

    I2C_init(&temp_sensor);

    I2C_master_receive_byte(temp_sensor.I2Cx, SLAVE_ADDRESS, &sensor_status);
    if (sensor_status & 0x18 != 0x18) {
        // initializing register 0x1B
        I2C_master_send_data(I2Cx, SLAVE_ADDRESS, register_clear, 3);
        delay_ms(5);
        I2C_master_receive_data(I2Cx, SLAVE_ADDRESS, received_bytes, 3);
        delay_ms(10);
        register_init[0] = 0xB0 | 0x1B; // register command
        register_init[1] = received_bytes[1];
        register_init[2] = received_bytes[2];
        I2C_master_send_data(I2Cx, SLAVE_ADDRESS, register_init, 3);

        // initializing register 0x1C
        register_clear[0] = 0x1C;
        I2C_master_send_data(I2Cx, SLAVE_ADDRESS, register_clear, 3);
        delay_ms(5);
        I2C_master_receive_data(I2Cx, SLAVE_ADDRESS, received_bytes, 3);
        delay_ms(10);
        register_init[0] = 0xB0 | 0x1C; // register command
        register_init[1] = received_bytes[1];
        register_init[2] = received_bytes[2];
        I2C_master_send_data(I2Cx, SLAVE_ADDRESS, register_init, 3);

        // initializing register 0x1E
        register_clear[0] = 0x1E;
        I2C_master_send_data(I2Cx, SLAVE_ADDRESS, register_clear, 3);
        delay_ms(5);
        I2C_master_receive_data(I2Cx, SLAVE_ADDRESS, received_bytes, 3);
        delay_ms(10);
        register_init[0] = 0xB0 | 0x1E; // register command
        register_init[1] = received_bytes[1];
        register_init[2] = received_bytes[2];
        I2C_master_send_data(I2Cx, SLAVE_ADDRESS, register_init, 3);
    }

}

void temp_sensor_measure(uint32_t* measurements) {
    uint8_t trigger_measurement[] = {0xAC, 0x33, 0x00}; // the command to trigger a measurement (0xAC) followed by it's 2 parameters (0x33 and 0x00)
    uint8_t status_word = 0x80;
    uint8_t read_values[6]; // first byte is the status word, second and third are humidity, fourth is humidity/temperature, fifth and sixth are temperature
    uint32_t humidity = 0;
    uint32_t temperature = 0;

    I2C_master_send_data(temp_sensor.I2Cx, SLAVE_ADDRESS, trigger_measurement, 3); // trigger a measurement
    delay_ms(80);
    while(status_word & 0x80 == 0x80) { // wait until the sensor is not busy
        I2C_master_receive_byte(temp_sensor.I2Cx, SLAVE_ADDRESS, &status_word);
        delay_ms(2);
    }
    I2C_master_receive_data(temp_sensor.I2Cx, SLAVE_ADDRESS, read_values, 6);

    humidity = (humidity|read_values[1]) << 8;
    humidity = (humidity|read_values[2]) << 8;
    humidity = (humidity | read_values[3]);
    humidity = humidity >> 4;
    humidity = humidity*100*10/1024/1024; // the humidity value multiplied by 10 (to get rid of the decimal)
    measurements[0] = humidity;

    temperature = (temperature|read_values[3]) << 8;
    temperature = (temperature|read_values[4]) << 8;
    temperature = (temperature|read_values[5]);
    temperature &= 0xFFFFF;
    temperature = temperature*200*10/1024/1024-500; // the temperature value multiplied by 10
    measurements[1] = temperature;
}


