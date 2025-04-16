

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"

#include "deck_spi.h"
#include "config.h"
#include "cfassert.h"
#include "nvicconf.h"
#define DEBUG_MODULE "DECK_SPI"
#include "debug_cf.h"


#define SPI_SCK_PIN CONFIG_SPI_PIN_CLK
#define SPI_MOSI_PIN CONFIG_SPI_PIN_MOSI
#define SPI_MISO_PIN CONFIG_SPI_PIN_MISO

#define DUMMY_BYTE 0xA5

static bool isInit = false;
spi_host_device_t host_id = SPI2_HOST;

static SemaphoreHandle_t spiMutex;

void spi_bus_begin(void)
{

    if (isInit) {
        return;
    }

    spiMutex = xSemaphoreCreateMutex();
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    }; //Defaults to 4094 if 0

    ret = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the pmw3901 to the SPI bus
    isInit = true;
}


esp_err_t spi_bus_device_add(spi_device_interface_config_t devcfg,spi_device_handle_t *spi)
{
    esp_err_t ret = ESP_OK;
    ret = spi_bus_add_device(host_id, &devcfg, spi);
    ESP_ERROR_CHECK(ret);
    return ret;
}


void spiBeginTransaction(uint32_t baudRatePrescaler)
{
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    // spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction()
{
    xSemaphoreGive(spiMutex);
}