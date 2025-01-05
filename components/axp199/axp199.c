/////////////////////////////////////////////////////////////////
/*

           __   _______ ___   ___ ___
     /\    \ \ / /  __ \__ \ / _ \__ \
    /  \    \ V /| |__) | ) | | | | ) |
   / /\ \    > < |  ___/ / /| | | |/ /
  / ____ \  / . \| |    / /_| |_| / /_
 /_/    \_\/_/ \_\_|   |____|\___/____|



MIT License

Copyright (c) 2019 lewis he
Copyright (c) 2025 FeOAr

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

---
### Original Project Information
axp20x.h - Arduino library for X-Power AXP202 chip.
Created by Lewis he on April 1, 2019.
github:https://github.com/lewisxhe/AXP202X_Libraries

---
### Modifications by FeOAr (2025)
- Added support for AXP199 power management chip.
- Refactored code to improve readability and maintainability.

*/
/////////////////////////////////////////////////////////////////

#include <string.h>
#include <math.h>
#include <esp_log.h>
#include "axp199.h"

static const char *AXP_TAG = "AXP199";

static uint8_t chgLEDParams[] = {
    0b00000000,  // OFF
    0b00010000,  // 1Hz
    0b00100000,  // 4Hz
    0b00110000   // ON
};

static uint8_t vbusLimitCurrent[] = {
    0b00000011,  // 100mA
    0b00000010,  // 500mA
    0b00000000,  // 900mA
};

static uint8_t startupParams[] = {
    0b00000000,  // 128ms
    0b01000000,  // 512ms
    0b10000000,  // 1s
    0b11000000   // 2s
};

static uint8_t longPressParams[] = {
    0b00000000,  // 1s
    0b00010000,  // 1.5s
    0b00100000,  // 2s
    0b00110000   // 2.5s
};

static uint8_t shutdownParams[] = {
    0b00000000,  // 4s
    0b00000001,  // 6s
    0b00000010,  // 8s
    0b00000011   // 10s
};

static uint8_t targetVolParams[] = {
    0b00000000,  // 4.1v
    0b00100000,  // 4.15
    0b01000000,  // 4.2
    0b01100000   // 4.36
};

/* -------------------------------- FUNCTION -------------------------------- */
static esp_err_t axp_read_byte(axp199_dev_t *axp199, uint8_t reg_addr, uint8_t nbytes, uint8_t *data)
{
    CHECK_ARG(axp199 && data);

    I2C_DEV_TAKE_MUTEX(&axp199->i2c_dev);
    I2C_DEV_CHECK(&axp199->i2c_dev, i2c_dev_read_reg(&axp199->i2c_dev, reg_addr, data, nbytes));
    I2C_DEV_GIVE_MUTEX(&axp199->i2c_dev);

    return ESP_OK;
}

static esp_err_t axp_write_byte(axp199_dev_t *axp199, uint8_t reg_addr, uint8_t nbytes, uint8_t *data)
{
    CHECK_ARG(axp199);
    I2C_DEV_TAKE_MUTEX(&axp199->i2c_dev);
    I2C_DEV_CHECK(&axp199->i2c_dev, i2c_dev_write_reg(&axp199->i2c_dev, reg_addr, data, nbytes));
    I2C_DEV_GIVE_MUTEX(&axp199->i2c_dev);

    return ESP_OK;
}

uint16_t axp_get_regist_H8_L5(axp199_dev_t *axp199, uint8_t reg_h8, uint8_t reg_l5)
{
    uint8_t val_high, val_low;
    ESP_ERROR_CHECK(axp_read_byte(axp199, reg_h8, 1, &val_high));
    ESP_ERROR_CHECK(axp_read_byte(axp199, reg_l5, 1, &val_low));
    
    return (val_high << 5) | (val_low & 0x1F);
}

uint16_t axp_get_regist_H8_L4(axp199_dev_t *axp199, uint8_t reg_h8, uint8_t reg_l4)
{
    uint8_t val_high, val_low;
    ESP_ERROR_CHECK(axp_read_byte(axp199, reg_h8, 1, &val_high));
    ESP_ERROR_CHECK(axp_read_byte(axp199, reg_l4, 1, &val_low));

    return (val_high << 4) | (val_low & 0x0F);
}

/* ----------------------------------- AXP ---------------------------------- */

esp_err_t axp199_dev_init(axp199_dev_t *axp199, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t pull_en, uint32_t clk_speed)
{
    CHECK_ARG(axp199);

    if (addr != AXP199_SLAVE_ADDRESS)
    {
        ESP_LOGE(AXP_TAG, "Invalid AXP I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    axp199->i2c_dev.port = port;
    axp199->i2c_dev.addr = addr;
    axp199->i2c_dev.cfg.sda_io_num = sda_gpio;
    axp199->i2c_dev.cfg.scl_io_num = scl_gpio;
    axp199->i2c_dev.cfg.sda_pullup_en = (pull_en & 0x02) >> 1;
    axp199->i2c_dev.cfg.scl_pullup_en = pull_en & 0x01;
    axp199->i2c_dev.cfg.master.clk_speed = clk_speed;
    axp199->i2c_dev.timeout_ticks = 0;
    axp199->irqStatus = 0;

    esp_err_t ret = 0;
    ret = i2c_dev_create_mutex(&axp199->i2c_dev);
    return  ret;
}

esp_err_t axp199_free_desc(axp199_dev_t *axp199)
{
    CHECK_ARG(axp199);
    return i2c_dev_delete_mutex(&axp199->i2c_dev);
}

/* --------------------------------- DCDC-1 --------------------------------- */

int32_t axp_get_dcdc1_voltage(axp199_dev_t *axp199)
{
    uint8_t val = 0;
    CHECK_LOGE(&axp199->i2c_dev, axp_read_byte(axp199, AXP199_DC1_VOL_SET, 1, &val), "Failed to read data");
    return val * 25 + 700;
}

int32_t axp_set_dcdc1_voltage(axp199_dev_t *axp199, uint32_t mv)
{
    if (mv < 700) {
        ESP_LOGW(AXP_TAG, "DCDC-1:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        ESP_LOGW(AXP_TAG, "DCDC-1:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_VOL_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_dcdc1_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    val = enable ? val | _BV(AXP199_DCDC1) : val & (~_BV(AXP199_DCDC1));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    return AXP_PASS;
}

bool axp_get_dcdc1_enable(axp199_dev_t *axp199)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    return IS_OPEN(val, AXP199_DCDC1);
}

/* --------------------------------- DCDC-2 --------------------------------- */

int32_t axp_get_dcdc2_voltage(axp199_dev_t *axp199)
{
    CHECK_ARG(axp199);
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC2_VOL_SET, 1, &val));
    return val * 25 + 700;
}

int32_t axp_set_dcdc2_voltage(axp199_dev_t *axp199, uint32_t mv)
{
    if (mv < 700) {
        ESP_LOGW(AXP_TAG, "DCDC-2:Below settable voltage:700mV~2275mV");
        mv = 700;
    }
    if (mv > 2275) {
        ESP_LOGW(AXP_TAG, "DCDC-2:Above settable voltage:700mV~2275mV");
        mv = 2275;
    }
    uint8_t val = (mv - 700) / 25;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC2_VOL_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_dcdc2_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val_0x10, val_0x12 = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_EXTEN_DC2_CTL, 1, &val_0x10));
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val_0x12));

    val_0x10 = enable ? val_0x10 | _BV(AXP199_CTL_DC2_BIT) : val_0x10 & (~_BV(AXP199_CTL_DC2_BIT));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_EXTEN_DC2_CTL, 1, &val_0x10));

    val_0x12 = enable ? val_0x12 | _BV(AXP199_DCDC2) : val_0x12 & (~_BV(AXP199_DCDC2));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val_0x12));

    return AXP_PASS;
}

bool axp_get_dcdc2_enable(axp199_dev_t *axp199)
{
    uint8_t val_0x10, val_0x12 = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_EXTEN_DC2_CTL, 1, &val_0x10));
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val_0x12));
    return IS_OPEN(val_0x10, AXP199_CTL_DC2_BIT) & IS_OPEN(val_0x12, AXP199_DCDC2);
}

/* --------------------------------- DCDC-3 --------------------------------- */

int32_t axp_get_dcdc3_voltage(axp199_dev_t *axp199)
{
    CHECK_ARG(axp199);
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC3_VOL_SET, 1, &val));
    return val * 25 + 700;
}

int32_t axp_set_dcdc3_voltage(axp199_dev_t *axp199, uint32_t mv)
{
    if (mv < 700) {
        ESP_LOGW(AXP_TAG, "DCDC-3:Below settable voltage:700mV~3500mV");
        mv = 700;
    }
    if (mv > 3500) {
        ESP_LOGW(AXP_TAG, "DCDC-3:Above settable voltage:700mV~3500mV");
        mv = 3500;
    }
    uint8_t val = (mv - 700) / 25;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC3_VOL_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_dcdc3_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));

    val = enable ? val | _BV(AXP199_DCDC3) : val & (~_BV(AXP199_DCDC3));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    return AXP_PASS;
}

bool axp_get_dcdc3_enable(axp199_dev_t *axp199)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    return IS_OPEN(val, AXP199_DCDC3);
}

/* ---------------------------------- EXTEN --------------------------------- */
int32_t axp_set_extern_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val_0x10, val_0x12 = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_EXTEN_DC2_CTL, 1, &val_0x10));
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val_0x12));

    val_0x10 = enable ? val_0x10 | _BV(AXP199_CTL_EXTEN_BIT) : val_0x10 & (~_BV(AXP199_CTL_EXTEN_BIT));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_EXTEN_DC2_CTL, 1, &val_0x10));

    val_0x12 = enable ? val_0x12 | _BV(AXP199_EXTEN) : val_0x12 & (~_BV(AXP199_EXTEN));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val_0x12));

    return AXP_PASS;
}

bool axp_get_extern_enable(axp199_dev_t *axp199)
{
    uint8_t val_0x10, val_0x12 = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_EXTEN_DC2_CTL, 1, &val_0x10));
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val_0x12));

    return IS_OPEN(val_0x10, AXP199_CTL_EXTEN_BIT) & IS_OPEN(val_0x12, AXP199_EXTEN);
}

/* ---------------------------------- LDO-2 --------------------------------- */

int32_t axp_get_ldo2_voltage(axp199_dev_t *axp199)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_LDO2_LDO3_VOL_SET, 1, &val));
    val &= 0xF0;
    val >>= 4;
    return val * 100 + 1800;
}

int32_t axp_set_ldo2_voltage(axp199_dev_t *axp199, uint32_t mv)
{
    uint8_t readValue, writeValue;
    if (mv < 1800) {
        ESP_LOGW(AXP_TAG, "LDO2:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }
    if (mv > 3300) {
        ESP_LOGW(AXP_TAG, "LDO2:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }
    writeValue = (mv - 1800) / 100;

    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_LDO2_LDO3_VOL_SET, 1, &readValue));
    readValue &= 0x0F;
    readValue |= (writeValue << 4);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_LDO2_LDO3_VOL_SET, 1, &readValue));
    return AXP_PASS;
}

int32_t axp_set_ldo2_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));

    val = enable ? val | _BV(AXP199_LDO2) : val & (~_BV(AXP199_LDO2));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));

    return AXP_PASS;
}


bool axp_get_ldo2_enable(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    return IS_OPEN(val, AXP199_LDO2);
}

/* ---------------------------------- LDO-3 --------------------------------- */

int32_t axp_get_ldo3_voltage(axp199_dev_t *axp199)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_LDO2_LDO3_VOL_SET, 1, &val));
    val &= 0x0F;
    return val * 100 + 1800;
}

int32_t axp_set_ldo3_voltage(axp199_dev_t *axp199, uint32_t mv)
{
    uint8_t readValue;
    if (mv < 1800) {
        ESP_LOGW(AXP_TAG, "LDO3:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }
    if (mv > 3300) {
        ESP_LOGW(AXP_TAG, "LDO3:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }

    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_LDO2_LDO3_VOL_SET, 1, &readValue));
    readValue &= 0xF0;
    readValue |= ((mv - 1800) / 100);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_LDO2_LDO3_VOL_SET, 1, &readValue));
    return AXP_PASS;
}

int32_t axp_set_ldo3_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));

    val = enable ? val | _BV(AXP199_LDO3) : val & (~_BV(AXP199_LDO3));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));

    return AXP_PASS;
}


bool axp_get_ldo3_enable(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC1_2_3_EXT_LDO2_3_CTL, 1, &val));
    return IS_OPEN(val, AXP199_LDO3);
}

/* -------------------------------- LDO-4(IO) ------------------------------- */

int32_t axp_get_ldo4_voltage(axp199_dev_t *axp199)
{
    uint8_t val = 0;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_GPIO0_LDO_VOL, 1, &val));
    val &= 0xF0;
    val >>= 4;
    return val * 100 + 1800;
}

int32_t axp_set_ldo4_voltage(axp199_dev_t *axp199, uint32_t mv)
{
    uint8_t readValue, writeValue;
    if (mv < 1800) {
        ESP_LOGW(AXP_TAG, "LDO4:Below settable voltage:1800mV~3300mV");
        mv = 1800;
    }
    if (mv > 3300) {
        ESP_LOGW(AXP_TAG, "LDO4:Above settable voltage:1800mV~3300mV");
        mv = 3300;
    }
    writeValue = (mv - 1800) / 100;
    readValue |= (writeValue << 4);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_GPIO0_LDO_VOL, 1, &readValue));
    return AXP_PASS;
}

int32_t axp_set_ldo4_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val = 0;
    val = enable ? val | _BV(AXP199_LDO_IO0) : val & (~_BV(AXP199_LDO_IO0));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_GPIO0_CTL, 1, &val));

    return AXP_PASS;
}


bool axp_get_ldo4_enable(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_GPIO0_CTL, 1, &val));
    if (val == 2)
    {
        return true;
    }else
    {
        return false; 
    }
}

/* ---------------------------- charge & battery ---------------------------- */

bool axp_get_chargeing_enable(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_CHARGE_CTL_1, 1, &val));
    if ((val & 0x80) >> 7 ) 
    {
        val = true;
    } else {
        val = false;
    }
    return val;
}

int32_t axp_set_chargeing_enable(axp199_dev_t *axp199, bool enable)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_CHARGE_CTL_1, 1, &val));
    val = enable ? (val | _BV(7)) : val & (~_BV(7));
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_CHARGE_CTL_1, 1, &val));
    return AXP_PASS;
}

bool axp_get_battery_connect(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_MODE_CHGSTATUS, 1, &val));
    return IS_OPEN(val, 5);
}

bool axp_get_chargeing_status(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_MODE_CHGSTATUS, 1, &val));
    return IS_OPEN(val, 6);
}

bool axp_get_vbus_plug_status(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_POWER_STATUS, 1, &val));
    return IS_OPEN(val, 5);
}

float axp_get_vbus_voltage(axp199_dev_t *axp199)
{
    float vol = axp_get_regist_H8_L4(axp199, AXP199_VBUS_VOL_ADC_H8, AXP199_VBUS_VOL_ADC_L4);
    vol *= AXP199_VBUS_VOLTAGE_STEP;
    return vol;
}

float axp_get_vbus_current(axp199_dev_t *axp199)
{
    float vol = axp_get_regist_H8_L4(axp199, AXP199_VBUS_CUR_ADC_H8, AXP199_VBUS_CUR_ADC_L4);
    vol *= AXP199_VBUS_CUR_STEP;
    return vol;
}

float axp_get_temperature(axp199_dev_t *axp199)
{
    // Internal temperature
    // 000H => -144.7℃
    // STEP => 0.1℃
    // FFFH => 264.8℃
    float temp = axp_get_regist_H8_L4(axp199, AXP199_INTERNAL_TEMP_ADC_H8, AXP199_INTERNAL_TEMP_ADC_L4);
    temp = temp*AXP199_INTERNAL_TEMP_STEP - 144.7;
    return temp;
}

float axp_get_ts_temperature(axp199_dev_t *axp199)
{
    float temp = axp_get_regist_H8_L4(axp199, AXP199_TS_IN_ADC_H8, AXP199_TS_IN_ADC_L4);
    temp *= AXP199_TS_PIN_OUT_STEP;
    return temp;   
}

/*
Note: the battery power formula:
Pbat =2* register value * Voltage LSB * Current LSB / 1000.
(Voltage LSB is 1.1mV; Current LSB is 0.5mA, and unit of calculation result is mW.)
*/
float axp_get_battery_inpower(axp199_dev_t *axp199)
{
    float rslt;
    uint8_t h_v, m_v, l_v;

    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_BAT_INS_POWER_H8, 1, &h_v));
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_BAT_INS_POWER_M8, 1, &m_v));
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_BAT_INS_POWER_L8, 1, &l_v));
    
    rslt = (h_v << 16) | (m_v << 8) | l_v;
    rslt = 2 * rslt * 1.1 * 0.5 / 1000;
    return rslt;
}

float axp_get_battery_voltage(axp199_dev_t *axp199)
{
    float vol = axp_get_regist_H8_L4(axp199, AXP199_BAT_AVER_VOL_H8, AXP199_BAT_AVER_VOL_L4);
    vol *= AXP199_BATT_VOLTAGE_STEP;
    return vol;
}

float axp_get_battery_charge_current(axp199_dev_t *axp199)
{
    float cur = axp_get_regist_H8_L5(axp199, AXP199_BAT_AVER_CHG_CUR_H8, AXP199_BAT_AVER_CHG_CUR_L5);
    cur *= AXP199_BATT_CHARGE_CUR_STEP;
    return cur;
}

float axp_get_battery_discharge_current(axp199_dev_t *axp199)
{
    float cur = axp_get_regist_H8_L5(axp199, AXP199_BAT_AVER_DISCHG_CUR_H8, AXP199_BAT_AVER_DISCHG_CUR_L5);
    cur *= AXP199_BATT_DISCHARGE_CUR_STEP;
    return cur;
}

float axp_get_sys_ipsout_voltage(axp199_dev_t *axp199)
{
    float vol = axp_get_regist_H8_L4(axp199, AXP199_APS_AVER_VOL_H8, AXP199_APS_AVER_VOL_L4);
    return vol;
}

int32_t axp_get_battery_charge_coulomb(axp199_dev_t *axp199)
{
    uint8_t chargCoulVal[4] = {0};
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_BAT_CHG_COULOMB_3, 4, chargCoulVal));
    return (chargCoulVal[0] << 24) + (chargCoulVal[1] << 16) + (chargCoulVal[2] << 8) + chargCoulVal[3];
}

int32_t axp_get_battery_discharge_coulomb(axp199_dev_t *axp199)
{
    uint8_t dischargCoulVal[4] = {0};
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_BAT_DISCHG_COULOMB_3, 4, dischargCoulVal));
    return (dischargCoulVal[0] << 24) + (dischargCoulVal[1] << 16) + (dischargCoulVal[2] << 8) + dischargCoulVal[3];
}

/*
Coulomb calculation formula:
C= 65536 * current LSB *（charge coulomb counter value - discharge coulomb counter value） /
3600 / ADC sample rate. Refer to REG84H setting for ADC sample rate；the current LSB is
0.5mA；unit of the calculation result is mAh. ）
*/

float axp_get_coulomb_data(axp199_dev_t *axp199)
{
    uint32_t charge = axp_get_battery_charge_coulomb(axp199);
    uint32_t discharge = axp_get_battery_discharge_coulomb(axp199);
    uint8_t rate = axp_get_adc_sampling_rate(axp199);
    float result = 65536.0 * 0.5 * ((float)charge - (float)discharge) / 3600.0 / rate;
    return result;
}

int32_t axp_set_charging_target_voltage(axp199_dev_t *axp199, axp_chargeing_vol_t param)
{
    if (param > AXP199_TARGET_VOL_4_36V)
    {
        return AXP_ARG_INVALID;
    }
    
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_CHARGE_CTL_1, 1, &val));
    val &= ~(0b01100000);
    val |= targetVolParams[param];
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_CHARGE_CTL_1, 1, &val));

    return AXP_PASS;
}

int32_t axp_adc1_enable(axp199_dev_t *axp199, axp_adc1_func_t params, bool enable)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_EN_1, 1, &val));
    if (enable)
        val |= params;
    else
        val &= ~params;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_ADC_EN_1, 1, &val));
    return AXP_PASS;
}

int32_t axp_adc2_enable(axp199_dev_t *axp199, axp_adc2_func_t params, bool enable)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_EN_2, 1, &val));
    if (enable)
        val |= params;
    else
        val &= ~params;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_ADC_EN_2, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_ts_current(axp199_dev_t *axp199, axp_ts_pin_current_t current)
{
    if (current > AXP_TS_PIN_CURRENT_80UA)
        return AXP_FAIL;
    
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    uint8_t rw = current;
    val &= 0xCF;
    val |= (rw << 4);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_ts_function(axp199_dev_t *axp199, axp_ts_pin_function_t func)
{
    if (func > AXP_TS_PIN_FUNCTION_ADC)
        return AXP_FAIL;
    
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    uint8_t rw = func;
    val &= 0xFB;
    val |= (rw << 2);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    return AXP_PASS;
}
int32_t axp_set_ts_mode(axp199_dev_t *axp199, axp_ts_pin_mode_t mode)
{
    if (mode > AXP_TS_PIN_MODE_ENABLE)
        return AXP_FAIL;
    
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    uint8_t rw = mode;
    val &= 0xFC;
    val |= rw;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
        
    // TS pin ADC function enable/disable
    if (mode == AXP_TS_PIN_MODE_DISABLE)
        axp_adc1_enable(axp199, AXP199_TS_PIN_ADC1, false);
    else
        axp_adc1_enable(axp199, AXP199_TS_PIN_ADC1, true);
    return AXP_PASS;
}

int32_t axp_set_timer(axp199_dev_t *axp199, uint8_t minutes)
{
    minutes |= 0x80;    //Clear timer flag
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_TIMER_CTL, 1, &minutes));
    return AXP_PASS;
}

int32_t axp_off_timer(axp199_dev_t *axp199)
{
    uint8_t minutes = 0x80;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_TIMER_CTL, 1, &minutes));
    return AXP_PASS;
}

int32_t axp_clear_timer_status(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_TIMER_CTL, 1, &val));
    val |= 0x80;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_TIMER_CTL, 1, &val));
    return AXP_PASS;
}

bool axp_get_timer_status(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_TIMER_CTL, 1, &val));
    return ( val & 0x80 ) >> 7;
}

int32_t axp_set_startup_time(axp199_dev_t *axp199, axp199_startup_time_t param)
{
    uint8_t val;
    if (param > ARRAY_SIZE(startupParams))
        return AXP_INVALID;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_PEK_SET, 1, &val));
    val &= (~0b11000000);
    val |= startupParams[param];
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_PEK_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_long_press_time(axp199_dev_t *axp199, axp_longPress_time_t param)
{
    uint8_t val;
    if (param > ARRAY_SIZE(longPressParams))
        return AXP_INVALID;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_PEK_SET, 1, &val));
    val &= (~0b00110000);
    val |= longPressParams[param];
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_PEK_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_shutdown_time(axp199_dev_t *axp199, axp_poweroff_time_t param)
{
    uint8_t val;
    if (param > ARRAY_SIZE(shutdownParams))
        return AXP_INVALID;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_PEK_SET, 1, &val));
    val &= (~0b00000011);
    val |= shutdownParams[param];
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_PEK_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_timeout_shutdown(axp199_dev_t *axp199, bool enable)
{
    uint8_t val;

    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_PEK_SET, 1, &val));
    if (enable)
        val |= (1 << 3);
    else
        val &= ~(1 << 3);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_PEK_SET, 1, &val));
    return AXP_PASS;
}

int32_t axp_shutdown(axp199_dev_t *axp199)
{
    uint8_t val;

    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
    val |= _BV(7);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_chg_led_mode(axp199_dev_t *axp199, axp_chgled_mode_t mode)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
    val &= 0b11001111;
    val |= _BV(3);
    switch (mode) {
    case AXP199_LED_OFF:
        val |= chgLEDParams[AXP199_LED_OFF];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
        break;
    case AXP199_LED_BLINK_1HZ:
        val |= chgLEDParams[AXP199_LED_BLINK_1HZ];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
        break;
    case AXP199_LED_BLINK_4HZ:
        val |= chgLEDParams[AXP199_LED_BLINK_4HZ];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
        break;
    case AXP199_LED_LOW_LEVEL:
        val |= chgLEDParams[AXP199_LED_LOW_LEVEL];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
        break;
    case AXP199_LED_CHARGE:
        val &= ~_BV(3);
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_OFF_BAT_CHGLED_CTL, 1, &val));
        break;
    default:
        return AXP_FAIL;
    }
    return AXP_PASS;
}

// int32_t getBattPercentage(void);
int32_t axp_set_adc_sampling_rate(axp199_dev_t *axp199, axp_adc_sampling_rate_t rate)
{
    if (rate > AXP_ADC_SAMPLING_RATE_200HZ)
        return AXP_FAIL;

    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    uint8_t rw = rate;
    val &= 0x3F;
    val |= (rw << 6);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    return AXP_PASS;   
}

uint8_t axp_get_adc_sampling_rate(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_ADC_SPEED_TS_CTL, 1, &val));
    return 25 * (int)pow(2, (val & 0xC0) >> 6);
}

//! coulomb
//! GPIO

uint16_t axp_get_power_down_voltage(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_VOFF_SET, 1, &val));
    val &= AXP199_VOFF_MASK;
    uint16_t voff = val * 100 + 2600;
    return voff;
}

// VOFF =[2.6+(Bit2-0)*0.1]V
int32_t axp_set_power_down_voltage(axp199_dev_t *axp199, uint16_t mv)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_VOFF_SET, 1, &val));
    val &= ~(AXP199_VOFF_MASK);
    val |= ((mv - 2600) / 100);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_VOFF_SET, 1, &val));
    return AXP_PASS;   
}

int32_t axp_set_current_limit_control(axp199_dev_t *axp199, axp199_limit_setting_t opt)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_VBUS_IPS_SET, 1, &val));
    val &= ~(AXP199_CUR_LIMIT_MASK);

    switch (opt)
    {
    case AXP199_VBUS_LIMIT_100MA:
        val |= vbusLimitCurrent[AXP199_VBUS_LIMIT_100MA];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_VBUS_IPS_SET, 1, &val));
        break;
    case AXP199_VBUS_LIMIT_500MA:
        val |= vbusLimitCurrent[AXP199_VBUS_LIMIT_500MA];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_VBUS_IPS_SET, 1, &val));
        break;
    case AXP199_VBUS_LIMIT_900MA:
        val |= vbusLimitCurrent[AXP199_VBUS_LIMIT_900MA];
        ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_VBUS_IPS_SET, 1, &val));
        break;
    default:
        return AXP_FAIL;
    }
    return AXP_PASS;   
}

int32_t axp_set_vol_warning_level_1(axp199_dev_t *axp199, uint16_t mv)
{
    uint8_t val = (mv / 0.0014 / 4 - 2.8672) / 1000.0;
    ESP_LOGD(AXP_TAG, "Set voltage warning Level 1 (Default: 0x68 now : 0x%x)\n", val);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_APS_LOW_WARNING_1, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_vol_warning_level_2(axp199_dev_t *axp199, uint16_t mv)
{
    uint8_t val = (mv / 0.0014 / 4 - 2.8672) / 1000.0;
    ESP_LOGD(AXP_TAG, "Set voltage warning Level 2 (Default: 0x5F now : 0x%x)\n", val);
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_APS_LOW_WARNING_2, 1, &val));
    return AXP_PASS;
}

uint16_t axp_get_vol_warning_level1(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_APS_LOW_WARNING_1, 1, &val));
    ESP_LOGD(AXP_TAG, "TarageVoltage:%.2f HEX:0x%x\n", 2.8672 + 0.0014 * val * 4.0, val);
    return ( 2.8672 + 0.0014 * val * 4.0) * 1000;
}

uint16_t axp_get_vol_warning_level2(axp199_dev_t *axp199)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_APS_LOW_WARNING_2, 1, &val));
    ESP_LOGD(AXP_TAG, "TarageVoltage:%.2f HEX:0x%x\n", 2.8672 + 0.0014 * val * 4.0, val);
    return ( 2.8672 + 0.0014 * val * 4.0) * 1000;
}

int32_t axp_set_dcdc_mode(axp199_dev_t *axp199, axp199_output_ctrl chn, axp199_dc_mode_t mode)
{
    uint8_t val;
    uint8_t bit;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DCDC_MODE_SET, 1, &val));

    switch (chn)
    {
        case AXP199_DCDC1: bit = 3; break;
        case AXP199_DCDC2: bit = 2; break;
        case AXP199_DCDC3: bit = 1; break;
        default: return AXP_FAIL;
    }

    val &= ~_BV(bit);

    if (mode != AXP199_DCDC_AUTO_MODE)
    {
        val |= _BV(bit);
    }

    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DCDC_MODE_SET, 1, &val));
    return AXP_PASS;   
}

axp199_dc_mode_t axp_get_dcdc_mode(axp199_dev_t *axp199, axp199_output_ctrl chn)
{
    uint8_t val;
    uint8_t bit;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DCDC_MODE_SET, 1, &val));

    switch (chn)
    {
        case AXP199_DCDC1: bit = 3; break;
        case AXP199_DCDC2: bit = 2; break;
        case AXP199_DCDC3: bit = 1; break;
        default: return AXP_FAIL;
    }

    if (val & _BV(bit))
    {
        return AXP199_DCDC_PWM_MODE;
    }
    else
    {
        return AXP199_DCDC_AUTO_MODE;
    }
}

int32_t axp_enable_dc2_vrc(axp199_dev_t *axp199, bool enable)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC2_VOL_DVM, 1, &val));
    val &= (~_BV(2));
    val |= enable;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC2_VOL_DVM, 1, &val));
    return AXP_PASS;
}

int32_t axp_set_dc2_vrc(axp199_dev_t *axp199, axp199_vrc_control_t opt)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_DC2_VOL_DVM, 1, &val));
    val &= (~_BV(0));
    val |= opt;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_DC2_VOL_DVM, 1, &val));
    return AXP_PASS;
}

//? Backup battery charge control

// Precharge timeout setting
int32_t axp_set_precharge_timeout(axp199_dev_t *axp199, axp199_precharge_timeout_t opt)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_CHARGE_CTL_2, 1, &val));
    val &= 0x3F;
    val |= opt;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_CHARGE_CTL_2, 1, &val));
    return AXP_PASS;
}

// Set timeout in constant current mode
int32_t axp_set_constant_current_timeout(axp199_dev_t *axp199, axp199_constant_current_t opt)
{
    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, AXP199_CHARGE_CTL_2, 1, &val));
    val &= 0xFC;
    val |= opt;
    ESP_ERROR_CHECK(axp_write_byte(axp199, AXP199_CHARGE_CTL_2, 1, &val));
    return AXP_PASS;
}

int32_t axp_enable_irq(axp199_dev_t *axp199, axp199_irq_t irq, bool enable)
{
    uint8_t base_addr = AXP199_INT_EN_1; // 中断使能寄存器的基地址

    // 根据传入的枚举值找到对应的寄存器
    uint8_t irq_index = 0; // 寄存器索引 (0 到 3)
    uint8_t irq_bit_pos = 0; // 寄存器内的位位置 (0 到 7)

    // 计算寄存器索引和位位置
    for (int i = 0; i < 4; ++i) {
        if (irq & (0xFFULL << (8 * i))) { // 判断中断位是否在当前字节内
            irq_index = i; // 当前寄存器的索引
            irq_bit_pos = (irq >> (8 * i)) & 0xFF; // 取得具体的位位置
            break;
        }
    }

    uint8_t val;
    ESP_ERROR_CHECK(axp_read_byte(axp199, base_addr + irq_index, 1, &val)); // 读取当前寄存器值

    if (enable)
        val |= irq_bit_pos; // 设置相应的中断位
    else
        val &= ~irq_bit_pos; // 清除相应的中断位

    ESP_LOGD(AXP_TAG, "%s [0x%x] val: 0x%x\n", enable ? "enable" : "disable", base_addr + irq_index, val);
    ESP_ERROR_CHECK(axp_write_byte(axp199, base_addr + irq_index, 1, &val)); // 写入更新后的寄存器值

    return AXP_PASS;
}

int32_t axp_read_irq(axp199_dev_t *axp199)
{
    uint8_t base_addr = AXP199_INT_STATUS_1;
    uint8_t buffer[sizeof(axp199->irqStatus)];
    memset(buffer, 0x00, sizeof(buffer));
    for (int16_t i = 0; i < sizeof(axp199->irqStatus); i++)
    {
        ESP_ERROR_CHECK(axp_read_byte(axp199, base_addr + i, 1, &buffer[i]));
    }
    memcpy(&axp199->irqStatus, buffer, sizeof(axp199->irqStatus));
    return AXP_PASS;
}

int32_t axp_clear_irq(axp199_dev_t *axp199)
{
    uint8_t base_addr = AXP199_INT_STATUS_1;
#if 0  // It's doesn't work...
    uint8_t buffer[sizeof(axp199->irqStatus)];
    memset(buffer, 0xFF, sizeof(buffer));
    ESP_ERROR_CHECK(axp_write_byte(axp199, base_addr, sizeof(buffer), buffer));
#else
    uint8_t val = 0xFF;
    for (int16_t i = 0; i < sizeof(axp199->irqStatus); i++)
    {
        ESP_ERROR_CHECK(axp_write_byte(axp199, base_addr+i, 1, &val));
    }
#endif
    axp199->irqStatus = 0;
    return AXP_PASS;
}

//= AXP199_INT_STATUS_1
bool axp_acin_over_voltage_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_ACIN_OVER_VOL_IRQ);
}

bool axp_acin_plug_in_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_ACIN_CONNECT_IRQ);
}

bool axp_acin_remove_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_ACIN_REMOVED_IRQ);
}

bool axp_vbus_over_voltage_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_OVER_VOL_IRQ);
}

bool axp_vbus_plug_in_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_CONNECT_IRQ);
}

bool axp_vbus_remove_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_REMOVED_IRQ);
}

bool axp_vbus_low_vhold_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_VHOLD_LOW_IRQ);
}

//= AXP199_INT_STATUS_2
bool axp_batt_plug_in_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_BATT_CONNECT_IRQ);
}

bool axp_batt_remove_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_BATT_REMOVED_IRQ);
}

bool axp_batt_enter_activate_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_BATT_ACTIVATE_IRQ);
}

bool axp_batt_exit_activate_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_BATT_EXIT_ACTIVATE_IRQ);
}

bool axp_charging_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_CHARGING_IRQ);
}

bool axp_charging_done_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_CHARGING_FINISHED_IRQ);
}

bool axp_batt_temp_high_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_BATT_OVER_TEMP_IRQ);
}

bool axp_batt_temp_low_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_BATT_LOW_TEMP_IRQ);
}


//= AXP199_INT_STATUS_3
bool axp_chip_overtemperature_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_CHIP_TEMP_HIGH_IRQ);
}

bool axp_charging_current_less_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_CHARGE_LOW_CUR_IRQ);
}

bool axp_dc1_voltage_less_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_DC1_LOW_VOL_IRQ);
}

bool axp_dc2_voltage_less_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_DC2_LOW_VOL_IRQ);
}

bool axp_dc3_voltage_less_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_DC3_LOW_VOL_IRQ);
}

bool axp_pek_short_press_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_PEK_SHORTPRESS_IRQ);
}

bool axp_pek_long_press_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_PEK_LONGPRESS_IRQ);
}

//= AXP199_INT_STATUS_4
bool axp_noe_power_on_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_NOE_ON_IRQ);
}

bool axp_noe_power_down_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_NOE_OFF_IRQ);
}

bool axp_vbus_effective_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_VAILD_IRQ);
}

bool axp_vbus_invalid_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_INVALID_IRQ);
}

bool axp_vubs_session_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_SESSION_AB_IRQ);
}

bool axp_vubs_session_end_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_VBUS_SESSION_END_IRQ);
}

bool axp_low_voltage_irq(axp199_dev_t *axp199)
{
    return (bool)(axp199->irqStatus & AXP199_APS_LOW_VOL_IRQ);
}

// TODO: 1. 主要功能测试
// TODO: 2. 现有库完整测试

