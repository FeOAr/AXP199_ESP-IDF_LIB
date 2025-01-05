/*
 * @Author: FeOAr feoar@outlook.com
 * @Date: 2024-05-02 21:32:05
 * @LastEditors: FeOAr feoar@outlook.com
 * @LastEditTime: 2024-08-25 18:36:33
 * @FilePath: \AXP199_DRV\main\main.c
 * @Description: 
 * 
 */
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <i2cdev.h>
#include <stdio.h>
#include <axp199.h>
#include <string.h>

#define CONFIG_I2C_AXP_SDA 11
#define CONFIG_I2C_AXP_SCL 10
#define CONFIG_I2C_PULL_UP 3
#define CONFIG_I2C_CLOCK_HZ 200000
static const char *EXAMPLE_TAG = "AXP_EXP";

axp199_dev_t axp_dev;

//i2cconfig --port=0 --freq=200000 --sda=11 --scl=10

void log_result(int16_t result, const char *success_msg, const char *failure_msg)
{
    if (result >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "%s", success_msg);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "%s Error Code: %d", failure_msg, result);
    }
}

/* -------------------------------------------------------------------------- */
/*                              power block test                              */
/* -------------------------------------------------------------------------- */

void axp_dcdc_1_test(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    int32_t vol = axp_get_dcdc1_voltage(dev);
    if (vol >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "Get axp dcdc1 voltage : %ld mV", vol);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to get axp dcdc1 voltage. Error Code: %ld", vol);
    }

    int16_t enable_result = axp_set_dcdc1_enable(dev, AXP_ENABLE);
    log_result(enable_result, "Enable dcdc1 success!", "Enable dcdc1 failed!");

    int16_t set_vol_result = axp_set_dcdc1_voltage(dev, 3000);
    log_result(set_vol_result, "Set dcdc1 voltage to 3000 mV success!", "Set dcdc1 voltage failed!");

    bool status = axp_get_dcdc1_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "Get dcdc1 status : %d", status);
}

void axp_dcdc_2_test(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    int32_t vol = axp_get_dcdc2_voltage(dev);
    if (vol >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "Get axp dcdc2 voltage : %ld mV", vol);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to get axp dcdc2 voltage. Error Code: %ld", vol);
    }

    int16_t enable_result = axp_set_dcdc2_enable(dev, AXP_ENABLE);
    log_result(enable_result, "Enable dcdc2 success!", "Enable dcdc2 failed!");

    int16_t set_vol_result = axp_set_dcdc2_voltage(dev, 2200);
    log_result(set_vol_result, "Set dcdc2 voltage to 2200 mV success!", "Set dcdc2 voltage failed!");

    bool status = axp_get_dcdc2_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "Get dcdc2 status : %d", status);
}

void axp_dcdc_3_test(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    int32_t vol = axp_get_dcdc3_voltage(dev);
    if (vol >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "Get axp dcdc3 voltage : %ld mV", vol);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to get axp dcdc3 voltage. Error Code: %ld", vol);
    }

    int16_t enable_result = axp_set_dcdc3_enable(dev, AXP_ENABLE);
    log_result(enable_result, "Enable dcdc3 success!", "Enable dcdc3 failed!");

    int16_t set_vol_result = axp_set_dcdc3_voltage(dev, 3300);
    log_result(set_vol_result, "Set dcdc3 voltage to 3300 mV success!", "Set dcdc3 voltage failed!");

    bool status = axp_get_dcdc3_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "Get dcdc3 status : %d", status);
}

void axp_ldo_2_test(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    // 获取 LDO2 电压
    int32_t ldo2_voltage = axp_get_ldo2_voltage(dev);
    if (ldo2_voltage >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "Get axp LDO2 voltage: %ld mV", ldo2_voltage);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to get axp LDO2 voltage. Error Code: %ld", ldo2_voltage);
    }

    // 设置 LDO2 电压为 2800 mV
    int16_t set_voltage_result = axp_set_ldo2_voltage(dev, 2800);
    log_result(set_voltage_result, "Set LDO2 voltage to 2800 mV success!", "Set LDO2 voltage failed!");

    // 启用 LDO2
    int16_t enable_result = axp_set_ldo2_enable(dev, AXP_ENABLE);
    log_result(enable_result, "Enable LDO2 success!", "Enable LDO2 failed!");

    // 获取 LDO2 的使能状态
    bool ldo2_status = axp_get_ldo2_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "Get LDO2 enable status: %d", ldo2_status);
}

void axp_ldo_3_test(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    // 获取 LDO3 电压
    int32_t ldo3_voltage = axp_get_ldo3_voltage(dev);
    if (ldo3_voltage >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "Get axp LDO3 voltage: %ld mV", ldo3_voltage);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to get axp LDO3 voltage. Error Code: %ld", ldo3_voltage);
    }

    // 设置 LDO3 电压为 2800 mV
    int16_t set_voltage_result = axp_set_ldo3_voltage(dev, 2800);
    log_result(set_voltage_result, "Set LDO3 voltage to 2800 mV success!", "Set LDO3 voltage failed!");

    // 启用 LDO3
    int16_t enable_result = axp_set_ldo3_enable(dev, AXP_ENABLE);
    log_result(enable_result, "Enable LDO3 success!", "Enable LDO3 failed!");

    // 获取 LDO3 的使能状态
    bool ldo3_status = axp_get_ldo3_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "Get LDO3 enable status: %d", ldo3_status);
}

void axp_ldo_4_test(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    // 获取 LDO4 电压
    int32_t ldo4_voltage = axp_get_ldo4_voltage(dev);
    if (ldo4_voltage >= 0)
    {
        ESP_LOGI(EXAMPLE_TAG, "Get axp LDO4 voltage: %ld mV", ldo4_voltage);
    }
    else
    {
        ESP_LOGE(EXAMPLE_TAG, "Failed to get axp LDO4 voltage. Error Code: %ld", ldo4_voltage);
    }

    // 设置 LDO4 电压为 3100 mV
    int16_t set_voltage_result = axp_set_ldo4_voltage(dev, 3100);
    log_result(set_voltage_result, "Set LDO4 voltage to 2800 mV success!", "Set LDO4 voltage failed!");

    // 启用 LDO4
    int16_t enable_result = axp_set_ldo4_enable(dev, AXP_ENABLE);
    log_result(enable_result, "Enable LDO4 success!", "Enable LDO4 failed!");

    // 获取 LDO4 的使能状态
    bool ldo4_status = axp_get_ldo4_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "Get LDO4 enable status: %d", ldo4_status);
}

/* -------------------------------------------------------------------------- */
/*                          charge&battery block test                         */
/* -------------------------------------------------------------------------- */
void test_charge_battery_vbus(axp199_dev_t *dev)
{
    ESP_LOGI(EXAMPLE_TAG, "============[%s]============", __FUNCTION__);
    bool status = false;
    int32_t ret = 0;
    float getValueFloat = 0.0;
    int32_t getValueInt = 0;

    ret = axp_adc1_enable(dev, AXP199_BATT_VOL_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_BATT_CUR_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_ACIN_VOL_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_ACIN_CUR_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_VBUS_VOL_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_VBUS_CUR_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_APS_VOL_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    ret = axp_adc1_enable(dev, AXP199_TS_PIN_ADC1, AXP_ENABLE);
    log_result(ret, "axp_adc1_enable success", "axp_adc1_enable failed");

    status = axp_get_chargeing_enable(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_chargeing_enable = %d", status);

    ret = axp_set_chargeing_enable(dev, AXP_ENABLE);
    log_result(ret, "chargeing enable success", "chargeing enable failed");

    status = axp_get_battery_connect(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_connect = %d", status);

    status = axp_get_chargeing_status(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_chargeing_status = %d", status);

    status = axp_get_vbus_plug_status(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_vbus_plug_status = %d", status);

    getValueFloat = axp_get_vbus_voltage(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_vbus_voltage = %f", getValueFloat);

    getValueFloat = axp_get_vbus_current(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_vbus_current = %f", getValueFloat);

    getValueFloat = axp_get_temperature(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_temperature = %f", getValueFloat);

    getValueFloat = axp_get_ts_temperature(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_ts_temperature = %f", getValueFloat);

    getValueFloat = axp_get_battery_inpower(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_inpower = %f", getValueFloat);

    getValueFloat = axp_get_battery_voltage(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_voltage = %f", getValueFloat);

    getValueFloat = axp_get_battery_charge_current(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_charge_current = %f", getValueFloat);

    getValueFloat = axp_get_battery_discharge_current(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_discharge_current = %f", getValueFloat);

    getValueFloat = axp_get_sys_ipsout_voltage(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_sys_ipsout_voltage = %f", getValueFloat);

    getValueFloat = axp_get_sys_ipsout_voltage(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_sys_ipsout_voltage = %f", getValueFloat);

    getValueInt = axp_get_battery_charge_coulomb(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_charge_coulomb = %ld", getValueInt);

    getValueInt = axp_get_battery_discharge_coulomb(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_battery_discharge_coulomb = %ld", getValueInt);

    getValueFloat = axp_get_coulomb_data(dev);
    ESP_LOGI(EXAMPLE_TAG, "axp_get_coulomb_data = %f", getValueFloat);

    ret = axp_set_charging_target_voltage(dev, AXP199_TARGET_VOL_4_1V);
    log_result(ret, "set_charging_target_voltage success", "set_charging_target_voltage failed");

    ret = axp_set_ts_current(dev, AXP_TS_PIN_CURRENT_80UA);
    log_result(ret, "axp_set_ts_current success", "axp_set_ts_current failed");

    ret = axp_set_startup_time(dev, AXP199_STARTUP_TIME_2S);
    log_result(ret, "axp_set_startup_time success", "axp_set_startup_time failed");

    ret = axp_set_long_press_time(dev, AXP_LONGPRESS_TIME_2S);
    log_result(ret, "axp_set_long_press_time success", "axp_set_long_press_time failed");

    ret = axp_set_shutdown_time(dev, AXP_POWER_OFF_TIME_4S);
    log_result(ret, "axp_set_shutdown_time success", "axp_set_shutdown_time failed");


    ret = axp_set_chg_led_mode(dev, AXP199_LED_CHARGE);
    log_result(ret, "axp_set_chg_led_mode success", "axp_set_chg_led_mode failed");

    ret = axp_shutdown(dev);
    log_result(ret, "axp_shutdown success", "axp_shutdown failed");
}

/* -------------------------------------------------------------------------- */
/*                            interrupt block test                            */
/* -------------------------------------------------------------------------- */

void task(void *ignore)
{
    memset(&axp_dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(axp199_dev_init(&axp_dev,
                                    AXP199_SLAVE_ADDRESS,
                                    0,
                                    CONFIG_I2C_AXP_SDA,
                                    CONFIG_I2C_AXP_SCL,
                                    CONFIG_I2C_PULL_UP,
                                    CONFIG_I2C_CLOCK_HZ));
    ESP_LOGI(EXAMPLE_TAG, "axp199 init done\n");

    /* --------------------------- function test part --------------------------- */
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGI(EXAMPLE_TAG, "------------------------------------------------------");
#if 0
    axp_dcdc_1_test(&axp_dev);
    axp_dcdc_2_test(&axp_dev);
    axp_dcdc_3_test(&axp_dev);

    axp_ldo_2_test(&axp_dev);
    axp_ldo_3_test(&axp_dev);
    axp_ldo_4_test(&axp_dev);
#endif

#if 0
    test_charge_battery_vbus(&axp_dev);
#endif

#if 1
    axp_clear_irq(&axp_dev);

#endif

    while (1)
    {
        //check irq status
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#define GPIO_INPUT_IO_9     GPIO_NUM_9
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_9)
static QueueHandle_t gpio_evt_queue = NULL;
#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) 
        {
            // printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            axp_read_irq(&axp_dev);
            printf("%s: axp199->irqStatus = %lx\n",EXAMPLE_TAG, axp_dev.irqStatus);
            axp_clear_irq(&axp_dev);
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
    
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    // gpio_set_intr_type(GPIO_INPUT_IO_9, GPIO_INTR_ANYEDGE);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_9, gpio_isr_handler, (void*) GPIO_INPUT_IO_9);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    // Init i2cdev library
    // Start task
    xTaskCreate(task, "axp_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}