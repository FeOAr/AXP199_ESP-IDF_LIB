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

#ifndef __AXP199_H__
#define __AXP199_H__

#include <i2cdev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RISING
#define RISING 0x01
#endif

#ifndef FALLING
#define FALLING 0x02
#endif

#ifdef _BV
#undef _BV
#endif
#define _BV(b)                                  (1ULL << (b))

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_LOGE(i2c_dev, x, msg, ...) do { \
        esp_err_t __; \
        if ((__ = x) != ESP_OK) { \
            I2C_DEV_GIVE_MUTEX(i2c_dev); \
            ESP_LOGE(AXP_TAG, msg, ## __VA_ARGS__); \
            return __; \
        } \
    } while (0)

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define AXP_ENABLE 1
#define AXP_DISABLE 0

//! Error Code
#define AXP_PASS                                (0)
#define AXP_FAIL                                (-1)
#define AXP_INVALID                             (-2)
#define AXP_NOT_INIT                            (-3)
#define AXP_NOT_SUPPORT                         (-4)
#define AXP_ARG_INVALID                         (-5)

//! Chip Address
#define AXP199_SLAVE_ADDRESS                    (0x34U)

//! REG MAP
// Group 1 - Power control
#define AXP199_POWER_STATUS                     (0x00)
#define AXP199_MODE_CHGSTATUS                   (0x01)
#define AXP199_OTG_STATUS                       (0x04)
#define AXP199_DATA_BUFFER1                     (0x06)
#define AXP199_DATA_BUFFER2                     (0x07)
#define AXP199_DATA_BUFFER3                     (0x08)
#define AXP199_DATA_BUFFER4                     (0x09)
#define AXP199_EXTEN_DC2_CTL                    (0x10)
#define AXP199_DC1_2_3_EXT_LDO2_3_CTL           (0x12)
#define AXP199_DC2_VOL_SET                      (0x23)
#define AXP199_DC2_VOL_DVM                      (0x25)
#define AXP199_DC1_VOL_SET                      (0x26)
#define AXP199_DC3_VOL_SET                      (0x27)
#define AXP199_LDO2_LDO3_VOL_SET                (0x28)
#define AXP199_VBUS_IPS_SET                     (0x30)
#define AXP199_VOFF_SET                         (0x31)
#define AXP199_OFF_BAT_CHGLED_CTL               (0x32)
#define AXP199_CHARGE_CTL_1                     (0x33)
#define AXP199_CHARGE_CTL_2                     (0x34)
#define AXP199_BACKUP_CHG                       (0x35)
#define AXP199_PEK_SET                          (0x36)
#define AXP199_DCDC_FREQ_SET                    (0x37)
#define AXP199_BAT_CHG_L_TEMP_ALARM_SET         (0x38)
#define AXP199_BAT_CHG_H_TEMP_ALARM_SET         (0x39)
#define AXP199_APS_LOW_WARNING_1                (0x3A)
#define AXP199_APS_LOW_WARNING_2                (0x3B)
#define AXP199_BAT_DISCHG_L_TEMP_ALARM_SET      (0x3C)
#define AXP199_BAT_DISCHG_H_TEMP_ALARM_SET      (0x3D)
#define AXP199_DCDC_MODE_SET                    (0x80)
#define AXP199_ADC_EN_1                         (0x82)
#define AXP199_ADC_EN_2                         (0x83)
#define AXP199_ADC_SPEED_TS_CTL                 (0x84)
#define AXP199_ADC_GPIO_INPUTRANGE              (0x85)
#define AXP199_TIMER_CTL                        (0x8A)
#define AXP199_VBUS_MONITOR_SET                 (0x8B)
#define AXP199_HOTOVER_CTL                      (0x8F)

// Group 2 - GPIO
#define AXP199_GPIO0_CTL                        (0x90)
#define AXP199_GPIO0_LDO_VOL                    (0x91)
#define AXP199_GPIO1_CTL                        (0x92)
#define AXP199_GPIO2_CTL                        (0x93)
#define AXP199_GPIO_012_SIG                     (0x94)
#define AXP199_GPIO_34_CTL                      (0x95)
#define AXP199_GPIO_34_SIG                      (0x96)
#define AXP199_GPIO_012_PULL_DOWN_CTL           (0x97)
#define AXP199_PWM_1_FREQ_SET                   (0x98)
#define AXP199_PWM_1_DUTY_RATIO_1_SET           (0x99)
#define AXP199_PWM_1_DUTY_RATIO_2_SET           (0x9A)
#define AXP199_PWM_2_FREQ_SET                   (0x9B)
#define AXP199_PWM_2_DUTY_RATIO_1_SET           (0x9C)
#define AXP199_PWM_2_DUTY_RATIO_2_SET           (0x9D)
#define AXP199_GPIO_5_CTL                       (0x9E)

// Group 3 - Irq control register
#define AXP199_INT_EN_1                         (0x40)
#define AXP199_INT_EN_2                         (0x41)
#define AXP199_INT_EN_3                         (0x42)
#define AXP199_INT_EN_4                         (0x43)
#define AXP199_INT_STATUS_1                     (0x44)
#define AXP199_INT_STATUS_2                     (0x45)
#define AXP199_INT_STATUS_3                     (0x46)
#define AXP199_INT_STATUS_4                     (0x47)

// Group 4 - ADC
#define AXP199_ACIN_VOL_ADC_H8                  (0x56)
#define AXP199_ACIN_VOL_ADC_L4                  (0x57)
#define AXP199_ACIN_CUR_ADC_H8                  (0x58)
#define AXP199_ACIN_CUR_ADC_L4                  (0x59)
#define AXP199_VBUS_VOL_ADC_H8                  (0x5A)
#define AXP199_VBUS_VOL_ADC_L4                  (0x5B)
#define AXP199_VBUS_CUR_ADC_H8                  (0x5C)
#define AXP199_VBUS_CUR_ADC_L4                  (0x5D)
#define AXP199_INTERNAL_TEMP_ADC_H8             (0x5E)
#define AXP199_INTERNAL_TEMP_ADC_L4             (0x5F)
#define AXP199_TS_IN_ADC_H8                     (0x62)
#define AXP199_TS_IN_ADC_L4                     (0x63)
#define AXP199_GPIO0_VOL_ADC_H8                 (0x64)
#define AXP199_GPIO0_VOL_ADC_L4                 (0x65)
#define AXP199_GPIO1_VOL_ADC_H8                 (0x66)
#define AXP199_GPIO1_VOL_ADC_L4                 (0x67)
#define AXP199_GPIO2_VOL_ADC_H8                 (0x68)
#define AXP199_GPIO2_VOL_ADC_L4                 (0x69)
#define AXP199_GPIO3_VOL_ADC_H8                 (0x6A)
#define AXP199_GPIO3_VOL_ADC_L4                 (0x6B)
#define AXP199_BAT_INS_POWER_H8                 (0x70)
#define AXP199_BAT_INS_POWER_M8                 (0x71)
#define AXP199_BAT_INS_POWER_L8                 (0x72)
#define AXP199_BAT_AVER_VOL_H8                  (0x78)
#define AXP199_BAT_AVER_VOL_L4                  (0x79)
#define AXP199_BAT_AVER_CHG_CUR_H8              (0x7A)
#define AXP199_BAT_AVER_CHG_CUR_L5              (0x7B)
#define AXP199_BAT_AVER_DISCHG_CUR_H8           (0x7C)
#define AXP199_BAT_AVER_DISCHG_CUR_L5           (0x7D)
#define AXP199_APS_AVER_VOL_H8                  (0x7E)
#define AXP199_APS_AVER_VOL_L4                  (0x7F)

#define AXP199_BAT_CHG_COULOMB_3                (0xB0)
#define AXP199_BAT_CHG_COULOMB_2                (0xB1)
#define AXP199_BAT_CHG_COULOMB_1                (0xB2)
#define AXP199_BAT_CHG_COULOMB_0                (0xB3)
#define AXP199_BAT_DISCHG_COULOMB_3             (0xB4)
#define AXP199_BAT_DISCHG_COULOMB_2             (0xB5)
#define AXP199_BAT_DISCHG_COULOMB_1             (0xB6)
#define AXP199_BAT_DISCHG_COULOMB_0             (0xB7)
#define AXP199_COULOMB_CTL                      (0xB8)


#define AXP199_CTL_DC2_BIT                      (0)
#define AXP199_CTL_EXTEN_BIT                    (2)
#define IS_OPEN(reg, channel)                   (bool)(reg & _BV(channel))

#define AXP199_BATT_VOLTAGE_STEP                (1.1F)
#define AXP199_BATT_DISCHARGE_CUR_STEP          (0.5F)
#define AXP199_BATT_CHARGE_CUR_STEP             (0.5F)
#define AXP199_VBUS_VOLTAGE_STEP                (1.7F)
#define AXP199_VBUS_CUR_STEP                    (0.375F)
#define AXP199_INTERNAL_TEMP_STEP               (0.1F)
#define AXP199_TS_PIN_OUT_STEP                  (0.8F)

#define AXP199_VOFF_MASK                        (0x07)
#define AXP199_CUR_LIMIT_MASK                   (0x03)

typedef enum {
    AXP199_DCDC1    = 0,
    AXP199_DCDC3    = 1,
    AXP199_LDO2     = 2,
    AXP199_LDO3     = 3,
    AXP199_DCDC2    = 4,
    AXP199_LDO_IO0  = 1,
    AXP199_EXTEN    = 6,
    AXP199_OUTPUT_MAX,
}axp199_output_ctrl;

typedef enum {
    AXP199_STARTUP_TIME_128MS,
    AXP199_STARTUP_TIME_512MS,
    AXP199_STARTUP_TIME_1S,
    AXP199_STARTUP_TIME_2S,
} axp199_startup_time_t;

typedef enum {
    AXP_LONGPRESS_TIME_1S,
    AXP_LONGPRESS_TIME_1S5,
    AXP_LONGPRESS_TIME_2S,
    AXP_LONGPRESS_TIME_2S5,
} axp_longPress_time_t;

typedef enum {
    AXP_POWER_OFF_TIME_4S,
    AXP_POWER_OFF_TIME_6S,
    AXP_POWER_OFF_TIME_8S,
    AXP_POWER_OFF_TIME_10S,
} axp_poweroff_time_t;

//REG 33H: Charging control 1 Charging target-voltage setting
typedef enum {
    AXP199_TARGET_VOL_4_1V,
    AXP199_TARGET_VOL_4_15V,
    AXP199_TARGET_VOL_4_2V,
    AXP199_TARGET_VOL_4_36V
} axp_chargeing_vol_t;

//REG 82H: ADC Enable 1 register Parameter
typedef enum {
    AXP199_BATT_VOL_ADC1    = _BV(7),
    AXP199_BATT_CUR_ADC1    = _BV(6),
    AXP199_ACIN_VOL_ADC1    = _BV(5),
    AXP199_ACIN_CUR_ADC1    = _BV(4),
    AXP199_VBUS_VOL_ADC1    = _BV(3),
    AXP199_VBUS_CUR_ADC1    = _BV(2),
    AXP199_APS_VOL_ADC1     = _BV(1),
    AXP199_TS_PIN_ADC1      = _BV(0)
} axp_adc1_func_t;

// REG 83H: ADC Enable 2 register Parameter
typedef enum {
    AXP199_TEMP_MONITORING_ADC2 = _BV(7),
    AXP199_GPIO0_FUNC_ADC2      = _BV(3),
    AXP199_GPIO1_FUNC_ADC2      = _BV(2),
    AXP199_GPIO2_FUNC_ADC2      = _BV(1),
    AXP199_GPIO3_FUNC_ADC2      = _BV(0)
} axp_adc2_func_t;

typedef enum {
    //! IRQ1 REG 40H
    AXP199_VBUS_VHOLD_LOW_IRQ       = _BV(1),   //VBUS is available, but lower than V HOLD, IRQ enable
    AXP199_VBUS_REMOVED_IRQ         = _BV(2),   //VBUS removed, IRQ enable
    AXP199_VBUS_CONNECT_IRQ         = _BV(3),   //VBUS connected, IRQ enable
    AXP199_VBUS_OVER_VOL_IRQ        = _BV(4),   //VBUS over-voltage, IRQ enable
    AXP199_ACIN_REMOVED_IRQ         = _BV(5),   //ACIN removed, IRQ enable
    AXP199_ACIN_CONNECT_IRQ         = _BV(6),   //ACIN connected, IRQ enable
    AXP199_ACIN_OVER_VOL_IRQ        = _BV(7),   //ACIN over-voltage, IRQ enable

    //! IRQ2 REG 41H
    AXP199_BATT_LOW_TEMP_IRQ        = _BV(8),   //Battery low-temperature, IRQ enable
    AXP199_BATT_OVER_TEMP_IRQ       = _BV(9),   //Battery over-temperature, IRQ enable
    AXP199_CHARGING_FINISHED_IRQ    = _BV(10),  //Charge finished, IRQ enable
    AXP199_CHARGING_IRQ             = _BV(11),  //Be charging, IRQ enable
    AXP199_BATT_EXIT_ACTIVATE_IRQ   = _BV(12),  //Exit battery activate mode, IRQ enable
    AXP199_BATT_ACTIVATE_IRQ        = _BV(13),  //Battery activate mode, IRQ enable
    AXP199_BATT_REMOVED_IRQ         = _BV(14),  //Battery removed, IRQ enable
    AXP199_BATT_CONNECT_IRQ         = _BV(15),  //Battery connected, IRQ enable

    //! IRQ3 REG 42H
    AXP199_PEK_LONGPRESS_IRQ        = _BV(16),  //PEK long press, IRQ enable
    AXP199_PEK_SHORTPRESS_IRQ       = _BV(17),  //PEK short press, IRQ enable
    //**Reserved and unchangeable BIT 2
    AXP199_DC3_LOW_VOL_IRQ          = _BV(19),  //DC-DC3output voltage is lower than the set value, IRQ enable
    AXP199_DC2_LOW_VOL_IRQ          = _BV(20),  //DC-DC2 output voltage is lower than the set value, IRQ enable
    AXP199_DC1_LOW_VOL_IRQ          = _BV(21),  //DC-DC1 output voltage is lower than the set value, IRQ enable
    AXP199_CHARGE_LOW_CUR_IRQ       = _BV(22),  //Charge current is lower than the set current, IRQ enable
    AXP199_CHIP_TEMP_HIGH_IRQ       = _BV(23),  //AXP202 internal over-temperature, IRQ enable

    //! IRQ4 REG 43H
    AXP199_APS_LOW_VOL_IRQ          = _BV(24),  //APS low-voltage, IRQ enable
    //**Reserved and unchangeable BIT 1
    AXP199_VBUS_SESSION_END_IRQ     = _BV(26),  //VBUS Session End IRQ enable
    AXP199_VBUS_SESSION_AB_IRQ      = _BV(27),  //VBUS Session A/B IRQ enable
    AXP199_VBUS_INVALID_IRQ         = _BV(28),  //VBUS invalid, IRQ enable
    AXP199_VBUS_VAILD_IRQ           = _BV(29),  //VBUS valid, IRQ enable
    AXP199_NOE_OFF_IRQ              = _BV(30),  //N_OE shutdown, IRQ enable
    AXP199_NOE_ON_IRQ               = _BV(31),  //N_OE startup, IRQ enable

    AXP199_ALL_IRQ                  = (0xFFFFFFFFULL)
} axp199_irq_t;

typedef enum {
    AXP199_LED_OFF,
    AXP199_LED_BLINK_1HZ,
    AXP199_LED_BLINK_4HZ,
    AXP199_LED_LOW_LEVEL,
    AXP199_LED_CHARGE
} axp_chgled_mode_t;

typedef enum {
    AXP_ADC_SAMPLING_RATE_25HZ  = 0,
    AXP_ADC_SAMPLING_RATE_50HZ  = 1,
    AXP_ADC_SAMPLING_RATE_100HZ = 2,
    AXP_ADC_SAMPLING_RATE_200HZ = 3,
} axp_adc_sampling_rate_t;

typedef enum {
    AXP_TS_PIN_CURRENT_20UA = 0,
    AXP_TS_PIN_CURRENT_40UA = 1,
    AXP_TS_PIN_CURRENT_60UA = 2,
    AXP_TS_PIN_CURRENT_80UA = 3,
} axp_ts_pin_current_t;

typedef enum {
    AXP_TS_PIN_FUNCTION_BATT    = 0,
    AXP_TS_PIN_FUNCTION_ADC     = 1,
} axp_ts_pin_function_t;

typedef enum {
    AXP_TS_PIN_MODE_DISABLE     = 0,
    AXP_TS_PIN_MODE_CHARGING    = 1,
    AXP_TS_PIN_MODE_SAMPLING    = 2,
    AXP_TS_PIN_MODE_ENABLE      = 3,
} axp_ts_pin_mode_t;

typedef enum {
    AXP_GPIO_0,
    AXP_GPIO_1,
    AXP_GPIO_2,
    AXP_GPIO_3,
    AXP_GPIO_4,
} axp_gpio_t;

typedef enum {
    AXP_IO_OUTPUT_LOW_MODE,
    AXP_IO_OUTPUT_HIGH_MODE,
    AXP_IO_INPUT_MODE,
    AXP_IO_LDO_MODE,
    AXP_IO_ADC_MODE,
    AXP_IO_FLOATING_MODE,
    AXP_IO_OPEN_DRAIN_OUTPUT_MODE,
    AXP_IO_PWM_OUTPUT_MODE,
    AXP_IO_EXTERN_CHARGING_CTRL_MODE,
} axp_gpio_mode_t;

typedef enum {
    AXP_IRQ_NONE,
    AXP_IRQ_RISING,
    AXP_IRQ_FALLING,
    AXP_IRQ_DOUBLE_EDGE,
} axp_gpio_irq_t;



typedef enum {
    AXP1XX_CHARGE_CUR_100MA,
    AXP1XX_CHARGE_CUR_190MA,
    AXP1XX_CHARGE_CUR_280MA,
    AXP1XX_CHARGE_CUR_360MA,
    AXP1XX_CHARGE_CUR_450MA,
    AXP1XX_CHARGE_CUR_550MA,
    AXP1XX_CHARGE_CUR_630MA,
    AXP1XX_CHARGE_CUR_700MA,
    AXP1XX_CHARGE_CUR_780MA,
    AXP1XX_CHARGE_CUR_880MA,
    AXP1XX_CHARGE_CUR_960MA,
    AXP1XX_CHARGE_CUR_1000MA,
    AXP1XX_CHARGE_CUR_1080MA,
    AXP1XX_CHARGE_CUR_1160MA,
    AXP1XX_CHARGE_CUR_1240MA,
    AXP1XX_CHARGE_CUR_1320MA,
} axp1xx_charge_current_t;


typedef enum {
    AXP199_VBUS_LIMIT_100MA,
    AXP199_VBUS_LIMIT_500MA,
    AXP199_VBUS_LIMIT_900MA
} axp199_limit_setting_t;

typedef enum {
    AXP199_DCDC_AUTO_MODE,
    AXP199_DCDC_PWM_MODE
} axp199_dc_mode_t;

/**
 * @brief  Voltage rise slope control
 */
typedef enum {
    AXP199_VRC_LEVEL0,  // 25mV/15.625us=1.6mV/us
    AXP199_VRC_LEVEL1,  //25mV/31.250us=0.8mV/us
} axp199_vrc_control_t;

typedef enum {
    AXP202_BACKUP_VOLTAGE_3V1,
    AXP202_BACKUP_VOLTAGE_3V0,
    AXP202_BACKUP_VOLTAGE_3V6,
    AXP202_BACKUP_VOLTAGE_2V5,
} axp202_backup_voltage_t;

typedef enum {
    AXP202_BACKUP_CURRENT_50UA,
    AXP202_BACKUP_CURRENT_100UA,
    AXP202_BACKUP_CURRENT_200UA,
    AXP202_BACKUP_CURRENT_400UA,
} axp202_backup_current_t;

typedef enum {
    AXP199_PRECHARGE_MINUTES_30,
    AXP199_PRECHARGE_MINUTES_40,
    AXP199_PRECHARGE_MINUTES_50,
    AXP199_PRECHARGE_MINUTES_60,
} axp199_precharge_timeout_t;

typedef enum {
    AXP199_CONSTANT_CUR_TIMEOUT_HOURS_7,
    AXP199_CONSTANT_CUR_TIMEOUT_HOURS_8,
    AXP199_CONSTANT_CUR_TIMEOUT_HOURS_9,
    AXP199_CONSTANT_CUR_TIMEOUT_HOURS_10,
} axp199_constant_current_t;

typedef struct
{
    i2c_dev_t i2c_dev;
    uint32_t irqStatus;
}axp199_dev_t;


/* -------------------------------------------------------------------------- */
/*                                Chip control                                */
/* -------------------------------------------------------------------------- */

esp_err_t axp199_dev_init(axp199_dev_t *axp199, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint8_t pull_en, uint32_t clk_speed);
esp_err_t axp199_free_desc(axp199_dev_t *axp199);

// Power Output Control
int32_t axp_set_power_out_put(uint8_t channel, bool enable);

/* -------------------------------------------------------------------------- */
/*                                Voltage control                             */
/* -------------------------------------------------------------------------- */
//= DCDC

int32_t axp_get_dcdc1_voltage(axp199_dev_t *axp199);
int32_t axp_set_dcdc1_voltage(axp199_dev_t *axp199, uint32_t mv);
int32_t axp_set_dcdc1_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_dcdc1_enable(axp199_dev_t *axp199);

int32_t axp_get_dcdc2_voltage(axp199_dev_t *axp199);
int32_t axp_set_dcdc2_voltage(axp199_dev_t *axp199, uint32_t mv);
int32_t axp_set_dcdc2_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_dcdc2_enable(axp199_dev_t *axp199);

int32_t axp_get_dcdc3_voltage(axp199_dev_t *axp199);
int32_t axp_set_dcdc3_voltage(axp199_dev_t *axp199, uint32_t mv);
int32_t axp_set_dcdc3_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_dcdc3_enable(axp199_dev_t *axp199);

int32_t axp_set_extern_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_extern_enable(axp199_dev_t *axp199);

//= LDO
int32_t axp_get_ldo2_voltage(axp199_dev_t *axp199);
int32_t axp_set_ldo2_voltage(axp199_dev_t *axp199, uint32_t mv);
int32_t axp_set_ldo2_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_ldo2_enable(axp199_dev_t *axp199);

int32_t axp_get_ldo3_voltage(axp199_dev_t *axp199);
int32_t axp_set_ldo3_voltage(axp199_dev_t *axp199, uint32_t mv);
int32_t axp_set_ldo3_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_ldo3_enable(axp199_dev_t *axp199);

int32_t axp_get_ldo4_voltage(axp199_dev_t *axp199);
int32_t axp_set_ldo4_voltage(axp199_dev_t *axp199, uint32_t mv);
int32_t axp_set_ldo4_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_ldo4_enable(axp199_dev_t *axp199);

/* -------------------------------------------------------------------------- */
/*                              charge & battery                              */
/* -------------------------------------------------------------------------- */
//= Power & Charge & battery
bool axp_get_chargeing_enable(axp199_dev_t *axp199);
int32_t axp_set_chargeing_enable(axp199_dev_t *axp199, bool enable);
bool axp_get_battery_connect(axp199_dev_t *axp199);
bool axp_get_chargeing_status(axp199_dev_t *axp199);
bool axp_get_vbus_plug_status(axp199_dev_t *axp199);

// float axp_get_acin_voltage(axp199_dev_t *axp199);
// float axp_get_acin_current(axp199_dev_t *axp199);
float axp_get_vbus_voltage(axp199_dev_t *axp199);
float axp_get_vbus_current(axp199_dev_t *axp199);
float axp_get_temperature(axp199_dev_t *axp199);
float axp_get_ts_temperature(axp199_dev_t *axp199);
// float axp_get_gpio0_voltage(axp199_dev_t *axp199);
// float axp_get_gpio1_voltage(axp199_dev_t *axp199);
float axp_get_battery_inpower(axp199_dev_t *axp199);
float axp_get_battery_voltage(axp199_dev_t *axp199);
float axp_get_battery_charge_current(axp199_dev_t *axp199);
float axp_get_battery_discharge_current(axp199_dev_t *axp199);
float axp_get_sys_ipsout_voltage(axp199_dev_t *axp199);
int32_t axp_get_battery_charge_coulomb(axp199_dev_t *axp199);
int32_t axp_get_battery_discharge_coulomb(axp199_dev_t *axp199);
float axp_get_coulomb_data(axp199_dev_t *axp199);
int32_t axp_set_charging_target_voltage(axp199_dev_t *axp199, axp_chargeing_vol_t param);

int32_t axp_adc1_enable(axp199_dev_t *axp199, axp_adc1_func_t params, bool enable);
int32_t axp_adc2_enable(axp199_dev_t *axp199, axp_adc2_func_t params, bool enable);
int32_t axp_set_ts_current(axp199_dev_t *axp199, axp_ts_pin_current_t current);
int32_t axp_set_ts_function(axp199_dev_t *axp199, axp_ts_pin_function_t func);
int32_t axp_set_ts_mode(axp199_dev_t *axp199, axp_ts_pin_mode_t mode);
int32_t axp_set_timer(axp199_dev_t *axp199, uint8_t minutes);
int32_t axp_off_timer(axp199_dev_t *axp199);
int32_t axp_clear_timer_status(axp199_dev_t *axp199);
bool axp_get_timer_status(axp199_dev_t *axp199);
int32_t axp_set_startup_time(axp199_dev_t *axp199, axp199_startup_time_t param);
int32_t axp_set_long_press_time(axp199_dev_t *axp199, axp_longPress_time_t param);
int32_t axp_set_shutdown_time(axp199_dev_t *axp199, axp_poweroff_time_t param);
int32_t axp_set_timeout_shutdown(axp199_dev_t *axp199, bool enable);
int32_t axp_shutdown(axp199_dev_t *axp199);


//! carefor
int32_t axp_set_chg_led_mode(axp199_dev_t *axp199, axp_chgled_mode_t mode);

// int32_t getBattPercentage(void);

int32_t axp_set_adc_sampling_rate(axp199_dev_t *axp199, axp_adc_sampling_rate_t rate);
uint8_t axp_get_adc_sampling_rate(axp199_dev_t *axp199);

//? coulomb
// uint8_t axp_get_coulomb_register(axp199_dev_t *axp199);
// float axp_get_coulomb_data(axp199_dev_t *axp199);
// int32_t axp_set_coulomb_register(uint8_t val);
// int32_t axp_enable_coulombcounter(axp199_dev_t *axp199);
// int32_t axp_disable_coulombcounter(axp199_dev_t *axp199);
// int32_t axp_stop_coulombcounter(axp199_dev_t *axp199);
// int32_t axp_clear_coulombcounter(axp199_dev_t *axp199);
//? coulomb
//? gpio
// int32_t axp_set_gpio_mode(axp_gpio_t gpio, axp_gpio_mode_t mode);
// int32_t axp_set_gpio_irq(axp_gpio_t gpio, axp_gpio_irq_t irq);
// int32_t axp_gpio_write(axp_gpio_t gpio, uint8_t val);
// int32_t axp_gpio_read(axp_gpio_t gpio);
//? gpio

uint16_t axp_get_power_down_voltage(axp199_dev_t *axp199);
int32_t axp_set_power_down_voltage(axp199_dev_t *axp199, uint16_t mv);
int32_t axp_set_current_limit_control(axp199_dev_t *axp199, axp199_limit_setting_t opt);

int32_t axp_set_vol_warning_level_1(axp199_dev_t *axp199, uint16_t mv);
int32_t axp_set_vol_warning_level_2(axp199_dev_t *axp199, uint16_t mv);
uint16_t axp_get_vol_warning_level1(axp199_dev_t *axp199);
uint16_t axp_get_vol_warning_level2(axp199_dev_t *axp199);

int32_t axp_set_dcdc_mode(axp199_dev_t *axp199, axp199_output_ctrl chn, axp199_dc_mode_t opt);
axp199_dc_mode_t axp_get_dcdc_mode(axp199_dev_t *axp199, axp199_output_ctrl chn);

int32_t axp_enable_dc2_vrc(axp199_dev_t *axp199, bool enable);
int32_t axp_set_dc2_vrc(axp199_dev_t *axp199, axp199_vrc_control_t opt);

//? Backup battery charge control
// int32_t axp_set_backup_charge_control(axp199_dev_t *axp199, bool enable);
// int32_t axp_set_backup_charge_voltage(axp199_dev_t *axp199, axp202_backup_voltage_t opt);
// int32_t axp_set_backup_charge_current(axp199_dev_t *axp199, axp202_backup_current_t opt);

// Precharge timeout setting
int32_t axp_set_precharge_timeout(axp199_dev_t *axp199, axp199_precharge_timeout_t opt);
// Set timeout in constant current mode
int32_t axp_set_constant_current_timeout(axp199_dev_t *axp199, axp199_constant_current_t opt);



//? gpio
// int32_t axp_irq_mask(axp199_dev_t *axp199, axp_gpio_irq_t irq);
// int32_t axp_set_gpio_interrupt(uint8_t *val, int32_t mode, bool en);
// int32_t axp199_gpio_set(axp_gpio_t gpio, axp_gpio_mode_t mode);
// int32_t axp199_gpio_0_select(axp_gpio_mode_t mode);
// int32_t axp199_gpio_1_select(axp_gpio_mode_t mode);
// int32_t axp199_gpio_2_select(axp_gpio_mode_t mode);
// int32_t axp199_gpio_3_select(axp_gpio_mode_t mode);
// int32_t axp199_gpio_irq_set(axp_gpio_t gpio, axp_gpio_irq_t irq);
// int32_t axp199_gpio_write(axp_gpio_t gpio, uint8_t val);
// int32_t axp199_gpio_read(axp_gpio_t gpio);
//? gpio

/* -------------------------------------------------------------------------- */
/*                              interrupt control                             */
/* -------------------------------------------------------------------------- */
//= IRQ
int32_t axp_enable_irq(axp199_dev_t *axp199, axp199_irq_t params, bool en);
int32_t axp_read_irq(axp199_dev_t *axp199);
int32_t axp_clear_irq(axp199_dev_t *axp199);

// ACIN overvoltage IRQ
bool axp_acin_over_voltage_irq(axp199_dev_t *axp199);
// ACIN access IRQ
bool axp_acin_plug_in_irq(axp199_dev_t *axp199);
// ACIN out of IRQ
bool axp_acin_remove_irq(axp199_dev_t *axp199);
// VBUS overvoltage IRQ
bool axp_vbus_over_voltage_irq(axp199_dev_t *axp199);
// VBUS access IRQ
bool axp_vbus_plug_in_irq(axp199_dev_t *axp199);
// VBUS shifted out of IRQ
bool axp_vbus_remove_irq(axp199_dev_t *axp199);
// VBUS is available but less than V HOLD IRQ
bool axp_vbus_low_vhold_irq(axp199_dev_t *axp199);

// Battery access IRQ
bool axp_batt_plug_in_irq(axp199_dev_t *axp199);
// Battery removed IRQ
bool axp_batt_remove_irq(axp199_dev_t *axp199);
// Battery activation mode IRQ
bool axp_batt_enter_activate_irq(axp199_dev_t *axp199);
// Exit battery activation mode IRQ
bool axp_batt_exit_activate_irq(axp199_dev_t *axp199);
// Charging IRQ
bool axp_charging_irq(axp199_dev_t *axp199);
// Charge complete IRQ
bool axp_charging_done_irq(axp199_dev_t *axp199);
// Battery temperature is too low IRQ
bool axp_batt_temp_high_irq(axp199_dev_t *axp199);
// Battery over temperature IRQ
bool axp_batt_temp_low_irq(axp199_dev_t *axp199);

// IC internal overheating IRQ
bool axp_chip_overtemperature_irq(axp199_dev_t *axp199);
// The charging current is less than the set current IRQ
bool axp_charging_current_less_irq(axp199_dev_t *axp199);
// DC-DC1 output voltage is less than the set value IRQ
bool axp_dc1_voltage_less_irq(axp199_dev_t *axp199);
// DC-DC2 output voltage is less than the set value IRQ
bool axp_dc2_voltage_less_irq(axp199_dev_t *axp199);
// DC-DC3 output voltage is less than the set value IRQ
bool axp_dc3_voltage_less_irq(axp199_dev_t *axp199);
// PEK short key IRQ
bool axp_pek_short_press_irq(axp199_dev_t *axp199);
// PEK long key IRQ
bool axp_pek_long_press_irq(axp199_dev_t *axp199);

// N_OE boot IRQ
bool axp_noe_power_on_irq(axp199_dev_t *axp199);
// N_OE shutdown IRQ
bool axp_noe_power_down_irq(axp199_dev_t *axp199);
// VBUS valid IRQ
bool axp_vbus_effective_irq(axp199_dev_t *axp199);
// VBUS invalid IRQ
bool axp_vbus_invalid_irq(axp199_dev_t *axp199);
// VBUS Session IRQ
bool axp_vubs_session_irq(axp199_dev_t *axp199);
// VBUS Session End IRQ
bool axp_vubs_session_end_irq(axp199_dev_t *axp199);
// APS low voltage IRQ (Set when below WL2, reset when above WL1)
bool axp_low_voltage_irq(axp199_dev_t *axp199);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __AXP199_H__ */