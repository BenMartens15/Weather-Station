/*
 * barometric_pressure_sensor_test.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "repo/lib/barometric_pressure_sensor/barometric_pressure_sensor.h"

void delay_ms(int time_ms);

int main() {
    delay_ms(1000);

    baro_sensor_init(SPI2, SPI_SS_PORT_E, SPI_SS_PIN_0);
}

void delay_ms(int time_ms) {
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

