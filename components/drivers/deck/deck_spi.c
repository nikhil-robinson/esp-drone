/*
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2015 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * deck_spi.c - Deck-API SPI communication implementation
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"

#include "deck_spi.h"
#include "deck_digital.h"
#include "config.h"
#include "cfassert.h"
#include "nvicconf.h"
#define DEBUG_MODULE "DECK_SPI"
#include "debug_cf.h"



#define DUMMY_BYTE 0xA5

static bool isInit = false;
static SemaphoreHandle_t spiMutex;

static void spiConfigureWithSpeed(uint32_t baudRatePrescaler);

static spi_device_handle_t spi;
static spi_device_handle_t bmi_spi = NULL;

void spiBegin(void)
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
        .max_transfer_sz = 4096*2,
    }; //Defaults to 4094 if 0
    spi_device_interface_config_t pwm_devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 3,
        .duty_cycle_pos = 128,  // default 128 = 50%/50% duty
        .cs_ena_pretrans = 0, // 0 not used
        .cs_ena_posttrans = 0,  // 0 not used
        .clock_speed_hz = 2000000,// 8,9,10,11,13,16,20,26,40,80
        .spics_io_num = SPI_CS2_PIN,
        .flags = 0,  // 0 not used
        .queue_size = 10,// transactionのキュー数。1以上の値を入れておく。
        .pre_cb = NULL,// transactionが始まる前に呼ばれる関数をセットできる
        .post_cb = NULL,// transactionが完了した後に呼ばれる関数をセットできる
    };

    spi_device_interface_config_t bmi_devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 128,  // default 128 = 50%/50% duty
        .cs_ena_pretrans = 0, // 0 not used
        .cs_ena_posttrans = 0,  // 0 not used
        .clock_speed_hz = 2000000,// 8,9,10,11,13,16,20,26,40,80
        .spics_io_num = SPI_CS1_PIN,
        .flags = 0,  // 0 not used
        .queue_size = 10,// transactionのキュー数。1以上の値を入れておく。
        .pre_cb = NULL,// transactionが始まる前に呼ばれる関数をセットできる
        .post_cb = NULL,// transactionが完了した後に呼ばれる関数をセットできる
    };

    pinMode(46, OUTPUT);//CSを設定
    digitalWrite(46, 1);//CSをHIGH
    pinMode(12, OUTPUT);//CSを設定
    digitalWrite(12, 1);//CSをHIGH
    vTaskDelay(5 / portTICK_PERIOD_MS);
    //Initialize the SPI bus
    spi_host_device_t host_id = SPI2_HOST;
    ret = spi_bus_initialize(host_id, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    //Attach the pmw3901 to the SPI bus
    ret = spi_bus_add_device(host_id, &pwm_devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(host_id, &bmi_devcfg, &bmi_spi);
    ESP_ERROR_CHECK(ret);

    isInit = true;
}

spi_device_handle_t spi_get_bmi_handle(void)
{
    return bmi_spi;
} 

static void spiConfigureWithSpeed(uint32_t baudRatePrescaler)
{
    //TODO:
}

bool spiTest(void)
{
    return isInit;
}

bool spiExchange(size_t length, bool is_tx, const uint8_t *data_tx, uint8_t *data_rx)
{
    if (isInit != true) {
        return false;
    }

    if (length == 0) {
        return true;    //no need to send anything
    }

    esp_err_t ret;

    if (is_tx == true) {

        static spi_transaction_t t;
        memset(&t, 0, sizeof(t));					//Zero out the transaction
        t.length = length * 8;						//Len is in bytes, transaction length is in bits.
        t.tx_buffer = data_tx;						//Data
        ret = spi_device_polling_transmit(spi, &t); //Transmit!
        assert(ret == ESP_OK);						//Should have had no issues.
        //DEBUG_PRINTD("spi send = %d",t.length);
        return true;
    }

    static spi_transaction_t r;
    memset(&r, 0, sizeof(r));
    r.length = length * 8;
    r.flags = SPI_TRANS_USE_RXDATA;
    ret = spi_device_polling_transmit(spi, &r);
    assert(ret == ESP_OK);

    if (r.rxlength > 0) {
        //DEBUG_PRINTD("rxlength = %d",r.rxlength);
        memcpy(data_rx, r.rx_data, length);
    }

    return true;
}

void spiBeginTransaction(uint32_t baudRatePrescaler)
{
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    spiConfigureWithSpeed(baudRatePrescaler);
}

void spiEndTransaction()
{
    xSemaphoreGive(spiMutex);
}
