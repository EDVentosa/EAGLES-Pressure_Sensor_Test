#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

//Sensor Config
#define PRESSURE_MIN_VOLTAGE    0.5f
#define PRESSURE_MAX_VOLTAGE    4.5f
#define PRESSURE_MAX_MPA        1.6f
#define VOLTAGE_DIVIDER_RATIO   1.5f

//I2C Config
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

//ADS Config
#define ADS1115_ADDRESS         0x48
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

static const char *TAG = "ADS1115";

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t ads_handle;

//Master Bus Init
void i2c_master_init(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &bus_handle)
    );
}

//ADS Init
void ads1115_init(void)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            bus_handle,
            &dev_config,
            &ads_handle
        )
    );
}

//Set write register
esp_err_t ads1115_write_register(uint8_t reg, uint16_t value)
{
    uint8_t data[3];

    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;
    data[2] = value & 0xFF;

    return i2c_master_transmit(
        ads_handle,
        data,
        sizeof(data),
        -1
    );
}

//Set read register
esp_err_t ads1115_read_register(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    esp_err_t ret = i2c_master_transmit_receive(
        ads_handle,
        &reg,
        1,
        data,
        2,
        -1
    );

    if (ret != ESP_OK) {
        return ret;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];

    return ESP_OK;
}

//Measure Voltage on AIN0
esp_err_t ads1115_read_voltage(float *voltage)
{
    uint16_t config = 0xC383;
    uint16_t raw_data;

    // Start ADC conversion
    ESP_ERROR_CHECK(
        ads1115_write_register(
            ADS1115_REG_CONFIG,
            config
        )
    );

    // Wait for conversion to finish
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read conversion register
    esp_err_t ret = ads1115_read_register(
        ADS1115_REG_CONVERSION,
        &raw_data
    );

    if (ret != ESP_OK) {
        return ret;
    }

    // Convert unsigned register representation to signed ADC value
    int16_t adc_value = (int16_t)raw_data;

    // Convert ADC value to voltage
    *voltage = adc_value * 0.000125f; //4.096/[(2^16)/2]

    return ESP_OK;
}

//Calculate sensor output voltage (before VDR)
float pressure_sensor_get_voltage(float adc_voltage)
{
    return adc_voltage * VOLTAGE_DIVIDER_RATIO;
}

//Convert voltage reading to MPa
float pressure_sensor_get_mpa(float sensor_voltage)
{
    return ((sensor_voltage - PRESSURE_MIN_VOLTAGE) /
            (PRESSURE_MAX_VOLTAGE - PRESSURE_MIN_VOLTAGE))
            * PRESSURE_MAX_MPA;
}

void app_main(void)
{
    i2c_master_init();
    ads1115_init();

    float adc_voltage;
    float sensor_voltage;
    float pressure_mpa;

    while (1)
    {
        esp_err_t ret = ads1115_read_voltage(&adc_voltage);

        if (ret == ESP_OK)
        {
            sensor_voltage = pressure_sensor_get_voltage(adc_voltage);      // Compensate for voltage divider
         
            pressure_mpa = pressure_sensor_get_mpa(sensor_voltage);         // Convert to pressure

            // Print measurements
            ESP_LOGI(
                TAG,
                "ADC: %.3f V | Sensor: %.3f V | Pressure: %.3f MPa",
                adc_voltage,
                sensor_voltage,
                pressure_mpa
            );
        }
        else
        {
            ESP_LOGE(TAG, "ADS1115 read failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}