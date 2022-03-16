/*
 * spi_test.c
 *
 *  Created on: Mar. 16, 2022
 *      Author: ben
 */

#include "repo/lib/drivers/spi_driver.h"

int main() {
    char commands[3];
    char message[12];

    sprintf(commands, "|-");
    sprintf(message, "Hello world");

    SPI_config_t SPI_test;
    SPI_test.SPI_device_mode = SPI_DEVICE_MODE_MASTER;
    SPI_test.SPI_cpol = SPI_CPOL_LOW;
    SPI_test.SPI_cpha = SPI_CPHA_LOW;
    SPI_test.SPI_speed = SPI_SCLK_SPEED_DIV64;
    SPI_test.SPI_dss = SPI_DSS_8BITS;

    SPI_init(SPI0, &SPI_test);
}
