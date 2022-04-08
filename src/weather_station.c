/*
 * weather_station.c
 *
 *  Created on: Mar. 24, 2022
 *      Author: ben
 */

#include <stdio.h>
#include <string.h>
#include "repo/lib/lcd/lcd.h"
#include "repo/lib/temp_humidity_sensor/temp_humidity_sensor.h"
#include "repo/lib/barometric_pressure_sensor/barometric_pressure_sensor.h"
#include "repo/lib/lux_sensor/lux_sensor.h"
#include "repo/lib/keypad/keypad.h"
#include "repo/lib/stepper_motor/stepper_motor.h"

#define STEPS_PER_REV   200

void delay_ms(uint16_t time_ms) {
    uint16_t i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}

int main() {
    uint8_t keypad_column_pins[] = {GPIO_PIN_NUM_3, GPIO_PIN_NUM_2, GPIO_PIN_NUM_4};
    uint8_t keypad_row_pins[] = {GPIO_PIN_NUM_7, GPIO_PIN_NUM_4, GPIO_PIN_NUM_5, GPIO_PIN_NUM_6};

    delay_ms(500);
    LCD_init(SPI3, SPI_SS_PORT_E, SPI_SS_PIN_1);
    temp_sensor_init(I2C1);
    baro_sensor_init(SPI2, SPI_SS_PORT_E, SPI_SS_PIN_0);
    lux_sensor_init(I2C1);
    keypad_init(GPIOA, GPIOC, keypad_column_pins, keypad_row_pins);
    stepper_init(200, GPIOE, GPIO_PIN_NUM_3, GPIOF, GPIO_PIN_NUM_1, GPIOF, GPIO_PIN_NUM_2, GPIOF, GPIO_PIN_NUM_3);

    delay_ms(1000);

    LCD_clear();

    LCD_display_string("Press 1, 2, 3,");
    LCD_move_cursor(1, 0);
    LCD_display_string("4, or 5.");

    while(1) {
        uint32_t humidity_temp[2]; // stores the humidity and temp values (multiplied by 10) read from the DHT20
        uint32_t ht_readings_integer[2]; // the integer portion of the humidity and temperature readings
        uint32_t ht_readings_decimal[2]; // the decimal portion of the humidity and temperature readings

        char display_output[16];
        memset(display_output, 0, sizeof(display_output)); // clear display_output
        if (get_keypad_key_pressed() == 1) {
            temp_sensor_measure(humidity_temp);
            temp_sensor_to_decimal(humidity_temp, ht_readings_integer, ht_readings_decimal);
            snprintf(display_output, 16, "Temp: %d.%d C", ht_readings_integer[1], ht_readings_decimal[1]);
            LCD_move_cursor(0, 0);
            LCD_clear();
            LCD_display_string(display_output);
            snprintf(display_output, 16, "RH: %d.%d%%", ht_readings_integer[0], ht_readings_decimal[0]);
            LCD_move_cursor(1, 0);
            LCD_display_string(display_output);
        }
        else if (get_keypad_key_pressed() == 2) {
            temp_sensor_measure(humidity_temp);
            temp_sensor_to_decimal(humidity_temp, ht_readings_integer, ht_readings_decimal);
            snprintf(display_output, 16, "Temp: %d.%d C", ht_readings_integer[1], ht_readings_decimal[1]);
            LCD_move_cursor(0, 0);
            LCD_clear();
            LCD_display_string(display_output);
        }
        else if(get_keypad_key_pressed() == 3) {
            temp_sensor_measure(humidity_temp);
            temp_sensor_to_decimal(humidity_temp, ht_readings_integer, ht_readings_decimal);
            snprintf(display_output, 16, "RH: %d.%d%%", ht_readings_integer[0], ht_readings_decimal[0]);
            LCD_move_cursor(0, 0);
            LCD_clear();
            LCD_display_string(display_output);
        }
        else if(get_keypad_key_pressed() == 4) {
            float barometric_pressure = baro_measure_pressure();
            snprintf(display_output, 16, "BP: %0.1fkPa", barometric_pressure);
            LCD_move_cursor(0, 0);
            LCD_clear();
            LCD_display_string(display_output);
        }
        else if(get_keypad_key_pressed() == 5) {
            float lux = lux_sensor_read_lux();
            snprintf(display_output, 16, "LUM: %0.1f lux", lux);
            LCD_move_cursor(0, 0);
            LCD_clear();
            LCD_display_string(display_output);
        }
        else if(get_keypad_key_pressed() == 6) {
            snprintf(display_output, 16, "Motor rotating");
            LCD_move_cursor(0, 0);
            LCD_clear();
            LCD_display_string(display_output);
            stepper_rotate(DIRECTION_CLOCKWISE, 5, 600);
        }
        delay_ms(250);
    }
}


