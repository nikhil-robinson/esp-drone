#include "ina3221.h"
#include "i2cdev.h"

static const char *TAG = "INA3221";

static esp_err_t ina3221_read_reg(I2C_Dev *dev, ina3221_addr_t addr, ina3221_reg_t reg, uint16_t *val) {
    uint8_t reg_addr = (uint8_t)reg;
    uint8_t data[2] = {0};
    esp_err_t ret = i2cdevReadReg8(I2C0_DEV, addr, reg_addr, (uint16_t)2, data);
    if (ret == ESP_OK) {
        *val = (data[0] << 8) | data[1];
    } 
    else 
    {
        *val =0;
    }
    return ret;
}

static esp_err_t ina3221_write_reg(I2C_Dev *dev, ina3221_addr_t addr, ina3221_reg_t reg, uint16_t val) {
    uint8_t data[2];
    data[0] = (val >> 8) & 0xFF;
    data[1] = val & 0xFF;
    esp_err_t ret = i2cdevWriteReg8(I2C0_DEV, addr,reg,(uint16_t) 2, data);
    return ret;
}

ina3221_config_t ina3221_init(I2C_Dev *dev, int sda_pin, int scl_pin, uint32_t clk_speed, ina3221_addr_t addr) {
    ina3221_config_t config = {
        .i2c_port = dev,
        .i2c_addr = addr,
        .shunt_res = {10, 10, 10},
        .filter_res = {0, 0, 0},
        .masken_reg = {0}
    };
    return config;
}

ina3221_config_t ina3221_set_shunt_res(ina3221_config_t config, uint32_t res_ch1, uint32_t res_ch2, uint32_t res_ch3) {
    ina3221_config_t new_config = config;
    new_config.shunt_res[0] = res_ch1;
    new_config.shunt_res[1] = res_ch2;
    new_config.shunt_res[2] = res_ch3;
    return new_config;
}

ina3221_config_t ina3221_set_filter_res(ina3221_config_t config, uint32_t res_ch1, uint32_t res_ch2, uint32_t res_ch3) {
    ina3221_config_t new_config = config;
    new_config.filter_res[0] = res_ch1;
    new_config.filter_res[1] = res_ch2;
    new_config.filter_res[2] = res_ch3;
    return new_config;
}

uint16_t ina3221_get_reg(ina3221_config_t config, ina3221_reg_t reg) {
    uint16_t val = 0;
    ina3221_read_reg(config.i2c_port, config.i2c_addr, reg, &val);
    return val;
}

esp_err_t ina3221_reset(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.reset = 1;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_mode_power_down(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_bus_en = 0;
    conf_reg.mode_continious_en = 0;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_mode_continuous(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_continious_en = 1;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_mode_triggered(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_continious_en = 0;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_shunt_meas_enable(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_shunt_en = 1;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_shunt_meas_disable(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_shunt_en = 0;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_bus_meas_enable(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_bus_en = 1;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_bus_meas_disable(ina3221_config_t config) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.mode_bus_en = 0;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_averaging_mode(ina3221_config_t config, ina3221_avg_mode_t mode) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.avg_mode = mode;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_bus_conversion_time(ina3221_config_t config, ina3221_conv_time_t conv_time) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.bus_conv_time = conv_time;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_shunt_conversion_time(ina3221_config_t config, ina3221_conv_time_t conv_time) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    conf_reg.shunt_conv_time = conv_time;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_pwr_valid_up_limit(ina3221_config_t config, int16_t voltage_mv) {
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_PWR_VALID_HI_LIM, (uint16_t)voltage_mv);
}

esp_err_t ina3221_set_pwr_valid_low_limit(ina3221_config_t config, int16_t voltage_mv) {
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_PWR_VALID_LO_LIM, (uint16_t)voltage_mv);
}

esp_err_t ina3221_set_shunt_sum_alert_limit(ina3221_config_t config, int32_t voltage_uv) {
    int16_t val = voltage_uv / 20;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_SHUNTV_SUM_LIM, (uint16_t)val);
}

esp_err_t ina3221_set_current_sum_alert_limit(ina3221_config_t config, int32_t current_ma) {
    int32_t shunt_uv = current_ma * (int32_t)config.shunt_res[INA3221_CH1];
    return ina3221_set_shunt_sum_alert_limit(config, shunt_uv);
}

ina3221_config_t ina3221_set_warn_alert_latch_enable(ina3221_config_t config) {
    masken_reg_t masken_reg;
    ina3221_config_t new_config = config;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret != ESP_OK) return new_config;
    masken_reg.warn_alert_latch_en = 1;
    ret = ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, *(uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

ina3221_config_t ina3221_set_warn_alert_latch_disable(ina3221_config_t config) {
    masken_reg_t masken_reg;
    ina3221_config_t new_config = config;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret != ESP_OK) return new_config;
    masken_reg.warn_alert_latch_en = 0;
    ret = ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, *(uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

ina3221_config_t ina3221_set_crit_alert_latch_enable(ina3221_config_t config) {
    masken_reg_t masken_reg;
    ina3221_config_t new_config = config;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret != ESP_OK) return new_config;
    masken_reg.crit_alert_latch_en = 1;
    ret = ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, *(uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

ina3221_config_t ina3221_set_crit_alert_latch_disable(ina3221_config_t config) {
    masken_reg_t masken_reg;
    ina3221_config_t new_config = config;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret != ESP_OK) return new_config;
    masken_reg.crit_alert_latch_en = 0;
    ret = ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, *(uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

ina3221_config_t ina3221_read_flags(ina3221_config_t config) {
    ina3221_config_t new_config = config;
    masken_reg_t masken_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

bool ina3221_get_timing_ctrl_alert_flag(ina3221_config_t config) {
    return config.masken_reg.timing_ctrl_alert;
}

bool ina3221_get_pwr_valid_alert_flag(ina3221_config_t config) {
    return config.masken_reg.pwr_valid_alert;
}

bool ina3221_get_current_sum_alert_flag(ina3221_config_t config) {
    return config.masken_reg.shunt_sum_alert;
}

bool ina3221_get_conversion_ready_flag(ina3221_config_t config) {
    return config.masken_reg.conv_ready;
}

uint16_t ina3221_get_manuf_id(ina3221_config_t config) {
    uint16_t id = 0;
    ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MANUF_ID, &id);
    return id;
}

uint16_t ina3221_get_die_id(ina3221_config_t config) {
    uint16_t id = 0;
    ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_DIE_ID, &id);
    return id;
}

esp_err_t ina3221_set_channel_enable(ina3221_config_t config, ina3221_ch_t channel) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    switch (channel) {
        case INA3221_CH1: conf_reg.ch1_en = 1; break;
        case INA3221_CH2: conf_reg.ch2_en = 1; break;
        case INA3221_CH3: conf_reg.ch3_en = 1; break;
    }
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_channel_disable(ina3221_config_t config, ina3221_ch_t channel) {
    conf_reg_t conf_reg;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, (uint16_t*)&conf_reg);
    if (ret != ESP_OK) return ret;
    switch (channel) {
        case INA3221_CH1: conf_reg.ch1_en = 0; break;
        case INA3221_CH2: conf_reg.ch2_en = 0; break;
        case INA3221_CH3: conf_reg.ch3_en = 0; break;
    }
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_CONF, *(uint16_t*)&conf_reg);
}

esp_err_t ina3221_set_warn_alert_shunt_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t voltage_uv) {
    ina3221_reg_t reg;
    switch (channel) {
        case INA3221_CH1: reg = INA3221_REG_CH1_WARNING_ALERT_LIM; break;
        case INA3221_CH2: reg = INA3221_REG_CH2_WARNING_ALERT_LIM; break;
        case INA3221_CH3: reg = INA3221_REG_CH3_WARNING_ALERT_LIM; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    int16_t val = voltage_uv / 5;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, reg, (uint16_t)val);
}

esp_err_t ina3221_set_crit_alert_shunt_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t voltage_uv) {
    ina3221_reg_t reg;
    switch (channel) {
        case INA3221_CH1: reg = INA3221_REG_CH1_CRIT_ALERT_LIM; break;
        case INA3221_CH2: reg = INA3221_REG_CH2_CRIT_ALERT_LIM; break;
        case INA3221_CH3: reg = INA3221_REG_CH3_CRIT_ALERT_LIM; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    int16_t val = voltage_uv / 5;
    return ina3221_write_reg(config.i2c_port, config.i2c_addr, reg, (uint16_t)val);
}

esp_err_t ina3221_set_warn_alert_current_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t current_ma) {
    int32_t shunt_uv = current_ma * (int32_t)config.shunt_res[channel];
    return ina3221_set_warn_alert_shunt_limit(config, channel, shunt_uv);
}

esp_err_t ina3221_set_crit_alert_current_limit(ina3221_config_t config, ina3221_ch_t channel, int32_t current_ma) {
    int32_t shunt_uv = current_ma * (int32_t)config.shunt_res[channel];
    return ina3221_set_crit_alert_shunt_limit(config, channel, shunt_uv);
}

ina3221_config_t ina3221_set_current_sum_enable(ina3221_config_t config, ina3221_ch_t channel) {
    masken_reg_t masken_reg;
    ina3221_config_t new_config = config;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret != ESP_OK) return new_config;
    switch (channel) {
        case INA3221_CH1: masken_reg.shunt_sum_en_ch1 = 1; break;
        case INA3221_CH2: masken_reg.shunt_sum_en_ch2 = 1; break;
        case INA3221_CH3: masken_reg.shunt_sum_en_ch3 = 1; break;
    }
    ret = ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, *(uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

ina3221_config_t ina3221_set_current_sum_disable(ina3221_config_t config, ina3221_ch_t channel) {
    masken_reg_t masken_reg;
    ina3221_config_t new_config = config;
    esp_err_t ret = ina3221_read_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, (uint16_t*)&masken_reg);
    if (ret != ESP_OK) return new_config;
    switch (channel) {
        case INA3221_CH1: masken_reg.shunt_sum_en_ch1 = 0; break;
        case INA3221_CH2: masken_reg.shunt_sum_en_ch2 = 0; break;
        case INA3221_CH3: masken_reg.shunt_sum_en_ch3 = 0; break;
    }
    ret = ina3221_write_reg(config.i2c_port, config.i2c_addr, INA3221_REG_MASK_ENABLE, *(uint16_t*)&masken_reg);
    if (ret == ESP_OK) new_config.masken_reg = masken_reg;
    return new_config;
}

int32_t ina3221_get_shunt_voltage(ina3221_config_t config, ina3221_ch_t channel) {
    ina3221_reg_t reg;
    switch (channel) {
        case INA3221_CH1: reg = INA3221_REG_CH1_SHUNTV; break;
        case INA3221_CH2: reg = INA3221_REG_CH2_SHUNTV; break;
        case INA3221_CH3: reg = INA3221_REG_CH3_SHUNTV; break;
        default: return 0;
    }
    uint16_t val_raw = 0;
    ina3221_read_reg(config.i2c_port, config.i2c_addr, reg, &val_raw);
    return (int16_t)val_raw * 5;
}

bool ina3221_get_warn_alert_flag(ina3221_config_t config, ina3221_ch_t channel) {
    switch (channel) {
        case INA3221_CH1: return config.masken_reg.warn_alert_ch1;
        case INA3221_CH2: return config.masken_reg.warn_alert_ch2;
        case INA3221_CH3: return config.masken_reg.warn_alert_ch3;
        default: return false;
    }
}

bool ina3221_get_crit_alert_flag(ina3221_config_t config, ina3221_ch_t channel) {
    switch (channel) {
        case INA3221_CH1: return config.masken_reg.crit_alert_ch1;
        case INA3221_CH2: return config.masken_reg.crit_alert_ch2;
        case INA3221_CH3: return config.masken_reg.crit_alert_ch3;
        default: return false;
    }
}

int32_t ina3221_estimate_offset_voltage(ina3221_config_t config, ina3221_ch_t channel, uint32_t bus_voltage) {
    const float bias_in = 10.0;   // uA
    const float r_in = 0.670;     // MOhm
    const uint32_t adc_step = 40; // uV
    float shunt_res = config.shunt_res[channel] / 1000.0; // Ohm
    float filter_res = config.filter_res[channel];        // Ohm
    float offset = (shunt_res + filter_res) * (bus_voltage / r_in + bias_in) - bias_in * filter_res;
    float remainder = (int32_t)offset % adc_step;
    if (remainder < adc_step / 2) {
        offset -= remainder;
    } else {
        offset += adc_step - remainder;
    }
    return (int32_t)offset;
}

float ina3221_get_current(ina3221_config_t config, ina3221_ch_t channel) {
    int32_t shunt_uv = ina3221_get_shunt_voltage(config, channel);
    return shunt_uv / (float)config.shunt_res[channel] / 1000.0;
}

float ina3221_get_current_compensated(ina3221_config_t config, ina3221_ch_t channel) {
    int32_t shunt_uv = ina3221_get_shunt_voltage(config, channel);
    float bus_v = ina3221_get_voltage(config, channel);
    int32_t offset_uv = ina3221_estimate_offset_voltage(config, channel, (uint32_t)(bus_v * 1000));
    return (shunt_uv - offset_uv) / (float)config.shunt_res[channel] / 1000.0;
}

float ina3221_get_voltage(ina3221_config_t config, ina3221_ch_t channel) {
    ina3221_reg_t reg;
    switch (channel) {
        case INA3221_CH1: reg = INA3221_REG_CH1_BUSV; break;
        case INA3221_CH2: reg = INA3221_REG_CH2_BUSV; break;
        case INA3221_CH3: reg = INA3221_REG_CH3_BUSV; break;
        default: return 0.0;
    }
    uint16_t val_raw = 0;
    ina3221_read_reg(config.i2c_port, config.i2c_addr, reg, &val_raw);
    return val_raw / 1000.0;
}