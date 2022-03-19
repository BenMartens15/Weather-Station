/*
 * i2c_test.c
 *
 *  Created on: Mar. 17, 2022
 *      Author: ben
 */

#include "repo/lib/temp_humidity_sensor/temp_humidity_sensor.h"

void delay(int time_ms) {
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

int main() {
    uint32_t temp_humidity[2];

    delay(1000);
    temp_sensor_init(I2C0);
    while(1) {
        temp_sensor_measure(temp_humidity);
        delay(3000);
    }
}

