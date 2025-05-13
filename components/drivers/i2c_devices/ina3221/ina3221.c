#include "ina3221.h"
#include "i2cdev.h"
#define DEBUG_MODULE "INA3221"
#include "debug_cf.h"

// I2C address
ina3221_addr_t _i2c_addr;

// Shunt resistance in mOhm
uint32_t _shuntRes[INA3221_CH_NUM];

// Series filter resistance in Ohm
uint32_t _filterRes[INA3221_CH_NUM];

// Value of Mask/Enable register.
masken_reg_t _masken_reg;

static I2C_Dev *I2Cx;


void INA3221_read(ina3221_reg_t reg, uint16_t *val) 
{
    uint8_t data[2];

    // Write register address
    bool ret = i2cdevReadReg8(I2Cx,_i2c_addr,reg,2,data);

    *val = (data[0] << 8) | data[1];
}

void INA3221_write(ina3221_reg_t reg, uint16_t *val) 
{
    uint8_t data[2];
    data[0] = (*val >> 8) & 0xFF;  // MSB
    data[1] = *val & 0xFF;         // LSB

    i2cdevWriteReg8(I2Cx, _i2c_addr,reg,2, data);
}

void INA3221_setAddr(ina3221_addr_t addr)
{
    _i2c_addr = addr;
}

void INA3221_begin(I2C_Dev *i2cPort,ina3221_addr_t addr) {

    _i2c_addr = addr;
    I2Cx = i2cPort;

    _shuntRes[0] = 10;
    _shuntRes[1] = 10;
    _shuntRes[2] = 10;

    _filterRes[0] = 0;
    _filterRes[1] = 0;
    _filterRes[2] = 0;
}

void INA3221_setShuntRes(uint32_t res_ch1, uint32_t res_ch2,
                          uint32_t res_ch3) {
    _shuntRes[0] = res_ch1;
    _shuntRes[1] = res_ch2;
    _shuntRes[2] = res_ch3;
}

void INA3221_setFilterRes(uint32_t res_ch1, uint32_t res_ch2,
                           uint32_t res_ch3) {
    _filterRes[0] = res_ch1;
    _filterRes[1] = res_ch2;
    _filterRes[2] = res_ch3;
}

uint16_t INA3221_getReg(ina3221_reg_t reg) {
    uint16_t val = 0;
    INA3221_read(reg, &val);
    return val;
}

void INA3221_reset() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.reset = 1;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setModePowerDown() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_bus_en        = 0;
    conf_reg.mode_continious_en = 0;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setModeContinious() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_continious_en = 1;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setModeTriggered() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_continious_en = 0;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setShuntMeasEnable() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_shunt_en = 1;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setShuntMeasDisable() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_shunt_en = 0;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setBusMeasEnable() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_bus_en = 1;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setBusMeasDisable() {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.mode_bus_en = 0;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setAveragingMode(ina3221_avg_mode_t mode) {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.avg_mode = mode;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setBusConversionTime(ina3221_conv_time_t convTime) {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.bus_conv_time = convTime;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setShuntConversionTime(ina3221_conv_time_t convTime) {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);
    conf_reg.shunt_conv_time = convTime;
    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setPwrValidUpLimit(int16_t voltagemV) {
    INA3221_write(INA3221_REG_PWR_VALID_HI_LIM, (uint16_t *)&voltagemV);
}

void INA3221_setPwrValidLowLimit(int16_t voltagemV) {
    INA3221_write(INA3221_REG_PWR_VALID_LO_LIM, (uint16_t *)&voltagemV);
}

void INA3221_setShuntSumAlertLimit(int32_t voltageuV) {
    int16_t val = 0;
    val         = voltageuV / 20;
    INA3221_write(INA3221_REG_SHUNTV_SUM_LIM, (uint16_t *)&val);
}

void INA3221_setCurrentSumAlertLimit(int32_t currentmA) {
    int16_t shuntuV = 0;
    shuntuV         = currentmA * (int32_t)_shuntRes[INA3221_CH1];
    INA3221_setShuntSumAlertLimit(shuntuV);
}

void INA3221_setWarnAlertLatchEnable() {
    masken_reg_t masken_reg;

    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    masken_reg.warn_alert_latch_en = 1;
    INA3221_write(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    _masken_reg = masken_reg;
}

void INA3221_setWarnAlertLatchDisable() {
    masken_reg_t masken_reg;

    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    masken_reg.warn_alert_latch_en = 1;
    INA3221_write(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    _masken_reg = masken_reg;
}

void INA3221_setCritAlertLatchEnable() {
    masken_reg_t masken_reg;

    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    masken_reg.crit_alert_latch_en = 1;
    INA3221_write(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    _masken_reg = masken_reg;
}

void INA3221_setCritAlertLatchDisable() {
    masken_reg_t masken_reg;

    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    masken_reg.crit_alert_latch_en = 1;
    INA3221_write(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    _masken_reg = masken_reg;
}

void INA3221_readFlags() {
    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&_masken_reg);
}

bool INA3221_getTimingCtrlAlertFlag() {
    return _masken_reg.timing_ctrl_alert;
}

bool INA3221_getPwrValidAlertFlag() {
    return _masken_reg.pwr_valid_alert;
}

bool INA3221_getCurrentSumAlertFlag() {
    return _masken_reg.shunt_sum_alert;
}

bool INA3221_getConversionReadyFlag() {
    return _masken_reg.conv_ready;
}

uint16_t INA3221_getManufID() {
    uint16_t id = 0;
    INA3221_read(INA3221_REG_MANUF_ID, &id);
    return id;
}

uint16_t INA3221_getDieID() {
    uint16_t id = 0;
    INA3221_read(INA3221_REG_DIE_ID, &id);
    return id;
}

void INA3221_setChannelEnable(ina3221_ch_t channel) {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);

    switch (channel) {
        case INA3221_CH1:
            conf_reg.ch1_en = 1;
            break;
        case INA3221_CH2:
            conf_reg.ch2_en = 1;
            break;
        case INA3221_CH3:
            conf_reg.ch3_en = 1;
            break;
        default:
            break;
    }

    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setChannelDisable(ina3221_ch_t channel) {
    conf_reg_t conf_reg;

    INA3221_read(INA3221_REG_CONF, (uint16_t *)&conf_reg);

    switch (channel) {
        case INA3221_CH1:
            conf_reg.ch1_en = 0;
            break;
        case INA3221_CH2:
            conf_reg.ch2_en = 0;
            break;
        case INA3221_CH3:
            conf_reg.ch3_en = 0;
            break;
        default:
            break;
    }

    INA3221_write(INA3221_REG_CONF, (uint16_t *)&conf_reg);
}

void INA3221_setWarnAlertShuntLimit(ina3221_ch_t channel, int32_t voltageuV) {
    ina3221_reg_t reg = INA3221_REG_CH2_WARNING_ALERT_LIM;
    int16_t val = 0;

    switch (channel) {
        case INA3221_CH1:
            reg = INA3221_REG_CH1_WARNING_ALERT_LIM;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_WARNING_ALERT_LIM;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_WARNING_ALERT_LIM;
            break;
            default:
            break;
    }

    val = voltageuV / 5;
    INA3221_write(reg, (uint16_t *)&val);
}

void INA3221_setCritAlertShuntLimit(ina3221_ch_t channel, int32_t voltageuV) {
    ina3221_reg_t reg = INA3221_REG_CH2_CRIT_ALERT_LIM;
    int16_t val = 0;

    switch (channel) {
        case INA3221_CH1:
            reg = INA3221_REG_CH1_CRIT_ALERT_LIM;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_CRIT_ALERT_LIM;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_CRIT_ALERT_LIM;
            break;
            default:
            break;
    }

    val = voltageuV / 5;
    INA3221_write(reg, (uint16_t *)&val);
}

void INA3221_setWarnAlertCurrentLimit(ina3221_ch_t channel,
                                       int32_t currentmA) {
    int32_t shuntuV = 0;
    shuntuV         = currentmA * (int32_t)_shuntRes[channel];
    INA3221_setWarnAlertShuntLimit(channel, shuntuV);
}

void INA3221_setCritAlertCurrentLimit(ina3221_ch_t channel,
                                       int32_t currentmA) {
    int32_t shuntuV = 0;
    shuntuV         = currentmA * (int32_t)_shuntRes[channel];
    INA3221_setCritAlertShuntLimit(channel, shuntuV);
}

void INA3221_setCurrentSumEnable(ina3221_ch_t channel) {
    masken_reg_t masken_reg;

    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);

    switch (channel) {
        case INA3221_CH1:
            masken_reg.shunt_sum_en_ch1 = 1;
            break;
        case INA3221_CH2:
            masken_reg.shunt_sum_en_ch2 = 1;
            break;
        case INA3221_CH3:
            masken_reg.shunt_sum_en_ch3 = 1;
            break;
            default:
            break;
    }

    INA3221_write(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    _masken_reg = masken_reg;
}

void INA3221_setCurrentSumDisable(ina3221_ch_t channel) {
    masken_reg_t masken_reg;

    INA3221_read(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);

    switch (channel) {
        case INA3221_CH1:
            masken_reg.shunt_sum_en_ch1 = 0;
            break;
        case INA3221_CH2:
            masken_reg.shunt_sum_en_ch2 = 0;
            break;
        case INA3221_CH3:
            masken_reg.shunt_sum_en_ch3 = 0;
            break;
            default:
            break;
    }

    INA3221_write(INA3221_REG_MASK_ENABLE, (uint16_t *)&masken_reg);
    _masken_reg = masken_reg;
}

int32_t INA3221_getShuntVoltage(ina3221_ch_t channel) {
    int32_t res;
    ina3221_reg_t reg;
    uint16_t val_raw = 0;

    switch (channel) {
        case INA3221_CH1:
            reg = INA3221_REG_CH1_SHUNTV;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_SHUNTV;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_SHUNTV;
            break;
        default:
            return 0;
    }

    INA3221_read(reg, &val_raw);

    // 1 LSB = 5uV
    res = (int16_t)val_raw * 5;

    return res;
}

bool INA3221_getWarnAlertFlag(ina3221_ch_t channel) {
    switch (channel) {
        case INA3221_CH1:
            return _masken_reg.warn_alert_ch1;
        case INA3221_CH2:
            return _masken_reg.warn_alert_ch2;
        case INA3221_CH3:
            return _masken_reg.warn_alert_ch3;
        default:
            return false;
    }
}

bool INA3221_getCritAlertFlag(ina3221_ch_t channel) {
    switch (channel) {
        case INA3221_CH1:
            return _masken_reg.crit_alert_ch1;
        case INA3221_CH2:
            return _masken_reg.crit_alert_ch2;
        case INA3221_CH3:
            return _masken_reg.crit_alert_ch3;
        default:
            return false;
    }
}

int32_t INA3221_estimateOffsetVoltage(ina3221_ch_t channel, uint32_t busV) {
    float bias_in     = 10.0;   // Input bias current at IN– in uA
    float r_in        = 0.670;  // Input resistance at IN– in MOhm
    uint32_t adc_step = 40;     // smallest shunt ADC step in uV
    float shunt_res   = _shuntRes[channel] / 1000.0;  // convert to Ohm
    float filter_res  = _filterRes[channel];
    int32_t offset    = 0.0;
    float reminder;

    offset = (shunt_res + filter_res) * (busV / r_in + bias_in) -
             bias_in * filter_res;

    // Round the offset to the closest shunt ADC value
    reminder = offset % adc_step;
    if (reminder < adc_step / 2) {
        offset -= reminder;
    } else {
        offset += adc_step - reminder;
    }

    return offset;
}

float INA3221_getCurrent(ina3221_ch_t channel) {
    int32_t shunt_uV = 0;
    float current_A  = 0;

    shunt_uV  = INA3221_getShuntVoltage(channel);
    current_A = shunt_uV / (int32_t)_shuntRes[channel] / 1000.0;
    return current_A;
}

float INA3221_getCurrentCompensated(ina3221_ch_t channel) {
    int32_t shunt_uV  = 0;
    int32_t bus_V     = 0;
    float current_A   = 0.0;
    int32_t offset_uV = 0;

    shunt_uV  = INA3221_getShuntVoltage(channel);
    bus_V     = INA3221_getVoltage(channel);
    offset_uV = INA3221_estimateOffsetVoltage(channel, bus_V);

    current_A = (shunt_uV - offset_uV) / (int32_t)_shuntRes[channel] / 1000.0;

    return current_A;
}

float INA3221_getVoltage(ina3221_ch_t channel) {
    float voltage_V = 0.0;
    ina3221_reg_t reg = INA3221_REG_CH2_BUSV;
    uint16_t val_raw = 0;

    switch (channel) {
        case INA3221_CH1:
            reg = INA3221_REG_CH1_BUSV;
            break;
        case INA3221_CH2:
            reg = INA3221_REG_CH2_BUSV;
            break;
        case INA3221_CH3:
            reg = INA3221_REG_CH3_BUSV;
            break;
            default:
            break;
    }

    INA3221_read(reg, &val_raw);

    voltage_V = val_raw / 1000.0;

    return voltage_V;
}