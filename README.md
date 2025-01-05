<!--
 * @Author: FeOAr feoar@outlook.com
 * @Date: 2024-05-02 21:32:05
 * @LastEditors: FeOAr feoar@outlook.com
 * @LastEditTime: 2025-01-05 11:01:24
 * @FilePath: \AXP199_DRV\README.md
 * @Description: 
 * 
-->
# 基于 ESP-IDF 的 AXP199 电源芯片库

本库基于 Arduino 下的 AXP20x 库开发，[AXP202X_Library](https://github.com/lewisxhe/AXP202X_Library)。

在开发其他 ESP-IDF 项目时，由于使用了 AXP199 芯片但缺乏专用驱动，因此编写了本驱动库。尽管 AXP 系列芯片的主要功能相似且具备通用性，但专用的库可以更好地满足实际需求。

欢迎在使用过程中提交问题或建议请提交 issue。您的反馈将帮助进一步完善本库。

项目中使用的I2C驱动库来自[esp-idf-lib](https://github.com/UncleRus/esp-idf-lib)


一个使用示例：
```c
static int32_t axp_power_init()
{
    int16_t enable_result = 0;
    int32_t set_vol_result = 0;
    memset(&axp_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(axp199_dev_init(&axp_dev,
                                    AXP199_SLAVE_ADDRESS,
                                    0,
                                    CONFIG_I2C_MASTER_SDA,
                                    CONFIG_I2C_MASTER_SCL,
                                    CONFIG_I2C_PULL_UP,
                                    CONFIG_I2C_CLOCK_HZ));
    ESP_LOGI(AXP_TAG, "axp199 init done\n");
    
    //  DCDC 1
    enable_result = axp_set_dcdc1_enable(&axp_dev, AXP_ENABLE);
    log_result(enable_result, "Enable dcdc1 success!", "Enable dcdc1 failed!");

    set_vol_result = axp_set_dcdc1_voltage(&axp_dev, 3300);
    log_result(set_vol_result, "Set dcdc1 voltage to 3300 mV success!", "Set dcdc1 voltage failed!");

    //  DCDC 2
    enable_result = axp_set_dcdc3_enable(&axp_dev, AXP_ENABLE);
    log_result(enable_result, "Enable dcdc3 success!", "Enable dcdc3 failed!");

    set_vol_result = axp_set_dcdc3_voltage(&axp_dev, 3300);
    log_result(set_vol_result, "Set dcdc3 voltage to 3300 mV success!", "Set dcdc3 voltage failed!");

    //  LDO 2
    set_vol_result = axp_set_ldo2_voltage(&axp_dev, 3300);
    log_result(set_vol_result, "Set LDO2 voltage to 3300 mV success!", "Set LDO2 voltage failed!");

    enable_result = axp_set_ldo2_enable(&axp_dev, AXP_ENABLE);
    log_result(enable_result, "Enable LDO2 success!", "Enable LDO2 failed!");


    //  LDO 3
    set_vol_result = axp_set_ldo3_voltage(&axp_dev, 3300);
    log_result(set_vol_result, "Set LDO3 voltage to 3300 mV success!", "Set LDO3 voltage failed!");

    enable_result = axp_set_ldo3_enable(&axp_dev, AXP_ENABLE);
    log_result(enable_result, "Enable LDO3 success!", "Enable LDO3 failed!");

    return ESP_OK;
}

```