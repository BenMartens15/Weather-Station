/*
 * spi_test.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "repo/lib/drivers/spi_driver.h"
#include <stdio.h>

void delay_ms(int time_ms);

int main() {
    char commands[3];
    char message[12];

    sprintf(commands, "|-");
    sprintf(message, "Hello world");

    SPI_config_t SPI_test;
    SPI_test.SPIx = SPI0;
    SPI_test.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    SPI_test.SPI_cpol = SPI_CPOL_LOW;
    SPI_test.SPI_cpha = SPI_CPHA_LOW;
    SPI_test.SPI_speed = SPI_SCLK_SPEED_DIV64;
    SPI_test.SPI_dss = SPI_DSS_8BITS;

    SPI_init(&SPI_test);
    SPI_ss_init(SPI_SS_PORT_F, SPI_SS_PIN_2);
    delay_ms(1000);
    SPI_write_byte(&SPI_test, 'B', SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 'e', SPI_SS_PORT_F, SPI_SS_PIN_2, 0);
    SPI_write_byte(&SPI_test, 'n', SPI_SS_PORT_F, SPI_SS_PIN_2, 1);
//    SPI_write_string(SPI0, &SPI_test, commands, SPI_SS_PORT_F, SPI_SS_PIN_2);
//    SPI_write_string(SPI0, &SPI_test, message, SPI_SS_PORT_F, SPI_SS_PIN_2);
}

void delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* execute NOP for 1ms */
}
