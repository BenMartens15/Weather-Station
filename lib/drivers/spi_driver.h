/*
 * spi_driver.h
 *
 *  Created on: Mar. 15, 2022
 *      Author: ben
 */

#ifndef REPO_LIB_DRIVERS_SPI_DRIVER_H_
#define REPO_LIB_DRIVERS_SPI_DRIVER_H_

#include "tm4c123gh6pm.h"
#include <stdint.h>

typedef struct {
    uint8_t SPI_device_mode;
    uint8_t SPI_cpol;
    uint8_t SPI_cpha;
    uint8_t SPI_speed;
    uint8_t SPI_dss;
}SPI_config_t;

void SPI_pclk_control(uint8_t SPIx, uint8_t enable_disable);
void SPI_init(uint8_t SPIx, SPI_config_t *pSPIConfig);

#define SPI0    0
#define SPI1    1
#define SPI2    2
#define SPI3    3

/*
 * @SPI_device_mode
 */
#define SPI_DEVICE_MODE_MASTER  0
#define SPI_DEVICE_MODE_SLAVE   1

/*
 * @SPI_cpol
 */
#define SPI_CPOL_HIGH           1
#define SPI_CPOL_LOW            0

/*
 * @SPI_cpha
 */
#define SPI_CPHA_HIGH           1
#define SPI_CPHA_LOW            0

/*
 * @SPI_speed
 */
#define SPI_SCLK_SPEED_DIV2     2
#define SPI_SCLK_SPEED_DIV4     4
#define SPI_SCLK_SPEED_DIV8     8
#define SPI_SCLK_SPEED_DIV16    16
#define SPI_SCLK_SPEED_DIV32    32
#define SPI_SCLK_SPEED_DIV64    64
#define SPI_SCLK_SPEED_DIV128   128

/*
 * @SPI_dss
 */
#define SPI_DSS_8BITS           0x08
#define SPI_DSS_16BITS          0x0F

#endif /* REPO_LIB_DRIVERS_SPI_DRIVER_H_ */
