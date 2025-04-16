#ifndef __SPIBUS_H__
#define __SPIBUS_H__



#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_err.h"
#include "driver/spi_master.h"

// Based on 84MHz peripheral clock
#define SPI_BAUDRATE_21MHZ  21*1000*1000
#define SPI_BAUDRATE_12MHZ  12*1000*1000
#define SPI_BAUDRATE_6MHZ   6*1000*1000
#define SPI_BAUDRATE_3MHZ   3*1000*1000
#define SPI_BAUDRATE_2MHZ   2*1000*1000

/**
 * Initialize the SPI.
 */
void spi_bus_begin(void);
esp_err_t spi_bus_device_add(spi_device_interface_config_t devcfg,spi_device_handle_t *spi);
void spiBeginTransaction(uint32_t baudRatePrescaler);
void spiEndTransaction();

#endif