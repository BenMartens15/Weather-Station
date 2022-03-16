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
    uint8_t SPIx;
    uint8_t SPI_device_mode;
    uint8_t SPI_cpol;
    uint8_t SPI_cpha;
    uint8_t SPI_speed;
    uint8_t SPI_dss;
}SPI_config_t;

void SPI_pclk_control(uint8_t SPIx, uint8_t enable_disable);
void SPI_init(SPI_config_t *pSPIConfig);
void SPI_ss_init(uint8_t port, uint8_t pin);

void SPI_write_byte(SPI_config_t *pSPIConfig, uint8_t data, uint8_t ss_port, uint8_t ss_pin, uint8_t release);
void SPI_write_string(SPI_config_t *pSPIConfig, char* data, uint8_t ss_port, uint8_t ss_pin);

#define SPI_PCLK_ENABLE     1
#define SPI_PCLK_DISABLE    0

/*
 * slave select ports
 */
#define SPI_SS_PORT_A   0
#define SPI_SS_PORT_B   1
#define SPI_SS_PORT_C   2
#define SPI_SS_PORT_D   3
#define SPI_SS_PORT_E   4
#define SPI_SS_PORT_F   5

/*
 * slave select pins
 */
#define SPI_SS_PIN_0    0
#define SPI_SS_PIN_1    1
#define SPI_SS_PIN_2    2
#define SPI_SS_PIN_3    3
#define SPI_SS_PIN_4    4
#define SPI_SS_PIN_5    5
#define SPI_SS_PIN_6    6
#define SPI_SS_PIN_7    7

/*
 * slave select pin states
 */
#define SPI_SS_HIGH     1
#define SPI_SS_LOW      0

/*
 * @SPIx
 */
#define SPI0                    0
#define SPI1                    1
#define SPI2                    2
#define SPI3                    3

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
#define SPI_DSS_8BITS           0x07
#define SPI_DSS_16BITS          0x0F

#endif /* REPO_LIB_DRIVERS_SPI_DRIVER_H_ */
