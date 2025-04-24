/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * led.c - LED handing functions
 */
#include <stdbool.h>

/*FreeRtos includes*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "led.h"
#include "stm32_legacy.h"
#include "led_strip.h"
#include "config.h"

// static unsigned int led_pin[] = {
//     [LED_BLUE] = LED_GPIO_BLUE,
//     [LED_RED]   = LED_GPIO_RED,
//     [LED_GREEN] = LED_GPIO_GREEN,
// };
// static int led_polarity[] = {
//     [LED_BLUE] = LED_POL_BLUE,
//     [LED_RED]   = LED_POL_RED,
//     [LED_GREEN] = LED_POL_GREEN,
// };

static bool isInit = false;
led_strip_handle_t led_strip;

#define LED_STRIP_RMT_RES_HZ (10 * 1000 * 1000)

void ledSetPixel(uint8_t red, uint8_t green, uint8_t blue);

// Initialize the green led pin as output
void ledInit()
{
    int i;

    if (isInit)
    {
        return;
    }

    led_strip_config_t strip_config = {
        .strip_gpio_num = CONFIG_LED_PIN,                                  // The GPIO that connected to the LED strip's data line
        .max_leds = 1,                                               // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,                               // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }};
  
    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 64,               // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }};
  
    // LED Strip object handle
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    isInit = true;
}

bool ledTest(void)
{
    ledSet(LED_GREEN, 1);
    ledSet(LED_RED, 0);
    vTaskDelay(M2T(250));
    ledSet(LED_GREEN, 0);
    ledSet(LED_RED, 1);
    vTaskDelay(M2T(250));
    // LED test end
    ledClearAll();
    ledSet(LED_BLUE, 1);

    return isInit;
}

void ledClearAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++)
    {
        // Turn off the LED:s
        ledSet(i, 0);
    }
}

void ledSetAll(void)
{
    int i;

    for (i = 0; i < LED_NUM; i++)
    {
        // Turn on the LED:s
        ledSet(i, 1);
    }
}
void ledSet(led_t led, bool value)
{
    uint16_t red =0,blue =0,green =0;
    switch (led)
    {
    case LED_BLUE:
    {
        blue = value ? 100:0;
        break;
    }
    case LED_RED:
    {
        red = value ? 100:0;
        break;
    }
    case LED_GREEN:
    {
        green = value ? 100:0;
        break;
    }
    default:
    {
        blue = value ? 100:0;
        red = value ? 100:0;
        green = value ? 100:0;
        break;
    }
    }
    ledSetPixel(red, green, blue);
}


void ledSetPixel(uint8_t red, uint8_t green, uint8_t blue)
{
    if (led_strip == NULL)
    {
        return;
    }
    
    led_strip_set_pixel(led_strip, 0, red, green, blue);
    led_strip_refresh(led_strip);
}