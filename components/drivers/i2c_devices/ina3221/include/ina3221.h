/*
    ESP-IDF library for INA3221 current and voltage sensor, written in functional programming style.

    MIT License

    Copyright (c) 2020 Beast Devices, Andrejs Bondarevs
    Adapted for ESP-IDF and functional programming in 2025.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
*/

#ifndef _INA3221_H_
#define _INA3221_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2cdev.h"

#define INA3221_CH_NUM 3

typedef enum {
    INA3221_ADDR40_GND = 0b1000000,  // A0 pin -> GND
    INA3221_ADDR41_VCC = 0b1000001,  // A0 pin -> VCC
    INA3221_ADDR42_SDA = 0b1000010,  // A0 pin -> SDA
    INA3221_ADDR43_SCL = 0b1000011   // A0 pin -> SCL
} ina3221_addr_t;

typedef enum {
    INA3221_CH1 = 0,
    INA3221_CH2,
    INA3221_CH3
} ina3221_ch_t;

typedef enum {
    INA3221_REG_CONF = 0,
    INA3221_REG_CH1_SHUNTV,
    INA3221_REG_CH1_BUSV,
    INA3221_REG_CH2_SHUNTV,
    INA3221_REG_CH2_BUSV,
    INA3221_REG_CH3_SHUNTV,
    INA3221_REG_CH3_BUSV,
    INA3221_REG_CH1_CRIT_ALERT_LIM,
    INA3221_REG_CH1_WARNING_ALERT_LIM,
    INA3221_REG_CH2_CRIT_ALERT_LIM,
    INA3221_REG_CH2_WARNING_ALERT_LIM,
    INA3221_REG_CH3_CRIT_ALERT_LIM,
    INA3221_REG_CH3_WARNING_ALERT_LIM,
    INA3221_REG_SHUNTV_SUM,
    INA3221_REG_SHUNTV_SUM_LIM,
    INA3221_REG_MASK_ENABLE,
    INA3221_REG_PWR_VALID_HI_LIM,
    INA3221_REG_PWR_VALID_LO_LIM,
    INA3221_REG_MANUF_ID = 0xFE,
    INA3221_REG_DIE_ID   = 0xFF
} ina3221_reg_t;

typedef enum {
    INA3221_REG_CONF_CT_140US = 0,
    INA3221_REG_CONF_CT_204US,
    INA3221_REG_CONF_CT_332US,
    INA3221_REG_CONF_CT_588US,
    INA3221_REG_CONF_CT_1100US,
    INA3221_REG_CONF_CT_2116US,
    INA3221_REG_CONF_CT_4156US,
    INA3221_REG_CONF_CT_8244US
} ina3221_conv_time_t;

typedef enum {
    INA3221_REG_CONF_AVG_1 = 0,
    INA3221_REG_CONF_AVG_4,
    INA3221_REG_CONF_AVG_16,
    INA3221_REG_CONF_AVG_64,
    INA3221_REG_CONF_AVG_128,
    INA3221_REG_CONF_AVG_256,
    INA3221_REG_CONF_AVG_512,
    INA3221_REG_CONF_AVG_1024
} ina3221_avg_mode_t;

typedef struct {
    uint16_t mode_shunt_en : 1;
    uint16_t mode_bus_en : 1;
    uint16_t mode_continious_en : 1;
    uint16_t shunt_conv_time : 3;
    uint16_t bus_conv_time : 3;
    uint16_t avg_mode : 3;
    uint16_t ch3_en : 1;
    uint16_t ch2_en : 1;
    uint16_t ch1_en : 1;
    uint16_t reset : 1;
} __attribute__((packed)) conf_reg_t;

typedef struct {
    uint16_t conv_ready : 1;
    uint16_t timing_ctrl_alert : 1;
    uint16_t pwr_valid_alert : 1;
    uint16_t warn_alert_ch3 : 1;
    uint16_t warn_alert_ch2 : 1;
    uint16_t warn_alert_ch1 : 1;
    uint16_t shunt_sum_alert : 1;
    uint16_t crit_alert_ch3 : 1;
    uint16_t crit_alert_ch2 : 1;
    uint16_t crit_alert_ch1 : 1;
    uint16_t crit_alert_latch_en : 1;
    uint16_t warn_alert_latch_en : 1;
    uint16_t shunt_sum_en_ch3 : 1;
    uint16_t shunt_sum_en_ch2 : 1;
    uint16_t shunt_sum_en_ch1 : 1;
    uint16_t reserved : 1;
} __attribute__((packed)) masken_reg_t;

typedef struct {
    I2C_Dev *i2c_port;
    ina3221_addr_t i2c_addr;
    uint32_t shunt_res[INA3221_CH_NUM]; // mOhm
    uint32_t filter_res[INA3221_CH_NUM]; // Ohm
    masken_reg_t masken_reg;
} ina3221_config_t;

// Initialize I2C and return initial config
ina3221_config_t ina3221_init(I2C_Dev *dev, int sda_pin, int scl_pin, uint32_t clk_speed, ina3221_addr_t addr);

// Set shunt resistor values
ina3221_config_t ina3221_set_shunt_res(ina3221_config_t config, uint32_t res_ch1, uint32_t res_ch2, uint32_t res_ch3);

// Set filter resistor values
ina3221_config_t ina3221_set_filter_res(ina3221_config_t config, uint32_t res_ch1, uint32_t res_ch2, uint32_t res_ch3);

// Read register value
uint16_t ina3221_get_reg(ina3221_config_t config, ina3221_reg_t reg);

// Reset device
esp_err_t ina3221_reset(ina3221_config_t config);

// Set power-down mode
esp_err_t ina3221_set_mode_power_down(ina3221_config_t config);

// Set continuous mode
esp_err_t ina3221_set_mode_continuous(ina3221_config_t config);

// Set triggered mode
esp_err_t ina3221_set_mode_triggered(ina3221_config_t config);

// Enable shunt voltage measurement
esp_err_t ina3221_set_shunt_meas_enable(ina3221_config_t config);

// Disable shunt voltage measurement
esp_err_t ina3221_set_shunt_meas_disable(ina3221_config_t config);

// Enable bus voltage measurement
esp_err_t ina3221_set_bus_meas_enable(ina3221_config_t config);

// Disable bus voltage measurement
esp_err_t ina3221_set_bus_meas_disable(ina3221_config_t config);

// Set averaging mode
esp_err_t ina3221_set_averaging_mode(ina3221_config_t config, ina3221_avg_mode_t mode);

// Set bus conversion time
esp_err_t ina3221_set_bus_conversion_time(ina3221_config_t config, ina3221_conv_time_t conv_time);

// Set shunt conversion time
esp_err_t ina3221_set_shunt_conversion_time(ina3221_config_t config, ina3221_conv_time_t conv_time);

// Set power-valid upper limit
esp_err_t ina3221_set_pwr_valid_up_limit(ina3221_config_t config, int16_t voltage_mv);

// Set power-valid lower limit
esp_err_t ina3221_set_pwr_valid_low_limit(ina3221_config_t config, int16_t voltage_mv);

// Set shunt sum alert limit
esp_err_t ina3221_set_shunt_sum_alert_limit(ina3221_config_t config, int32_t voltage_uv);

// Set current sum alert limit
esp_err_t ina3221_set_current_sum_alert_limit(ina3221_config_t config, int32_t current_ma);

// Enable warning alert latch
ina3221_config_t ina3221_set_warn_alert_latch_enable(ina3221_config_t config);

// Disable warning alert latch
ina3221_config_t ina3221_set_warn_alert_latch_disable(ina3221_config_t config);

// Enable critical alert latch
ina3221_config_t ina3221_set_crit_alert_latch_enable(ina3221_config_t config);

// Disable critical alert latch
ina3221_config_t ina3221_set_crit_alert_latch_disable(ina3221_config_t config);

// Read flags and update config
ina3221_config_t ina3221_read_flags(ina3221_config_t config);

// Get timing control alert flag
bool ina3221_get_timing_ctrl_alert_flag(ina3221_config_t config);

// Get power valid alert flag
bool ina3221_get_pwr_valid_alert_flag(ina3221_config_t config);

// Get current sum alert flag
bool ina3221_get_current_sum_alert_flag(ina3221_config_t config);

// Get conversion ready flag
bool ina3221_get_conversion_ready_flag(ina3221_config_t config);

// Get manufacturer ID
uint16_t ina3221_get_manuf_id(ina3221_config_t config);

// Get die ID
uint16_t ina3221_get_die_id(ina3221_config_t config);

// Enable channel measurements
esp_err_t ina3221_set_channel_enable(ina3221_config_t config, ina3221_ch_t channel);

// Disable channel measurements
esp_err_t ina3221_set_channel_disable(ina3221_config_t config, ina3221_ch_t channel);

// Set warning alert shunt voltage limit
esp_err_t ina3221_set_warn_alert_shunt_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t voltage_uv);

// Set critical alert shunt voltage limit
esp_err_t ina3221_set_crit_alert_shunt_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t voltage_uv);

// Set warning alert current limit
esp_err_t ina3221_set_warn_alert_current_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t current_ma);

// Set critical alert current limit
esp_err_t ina3221_set_crit_alert_current_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t current_ma);

// Enable current sum for channel
ina3221_config_t ina3221_set_current_sum_enable(ina3221_config_t config, ina3221_ch_t channel);

// Disable current sum for channel
ina3221_config_t ina3221_set_current_sum_disable(ina3221_config_t config, ina3221_ch_t channel);

// Get shunt voltage in uV
int32_t ina3221_get_shunt_voltage(ina3221_config_t config, ina3221_ch_t channel);

// Get warning alert flag
bool ina3221_get_warn_alert_flag(ina3221_config_t config, ina3221_ch_t channel);

// Get critical alert flag
bool ina3221_get_crit_alert_flag(ina3221_config_t config, ina3221_ch_t channel);

// Estimate offset voltage
int32_t ina3221_estimate_offset_voltage(ina3221_config_t config, ina3221_ch_t channel, uint32_t bus_voltage);

// Get current in A
float ina3221_get_current(ina3221_config_t config, ina3221_ch_t channel);

// Get compensated current in A
float ina3221_get_current_compensated(ina3221_config_t config, ina3221_ch_t channel);

// Get bus voltage in V
float ina3221_get_voltage(ina3221_config_t config, ina3221_ch_t channel);

#endif