/**
 * @file basic_reading.c
 * @brief Basic DS1603L sensor reading example
 *
 * This example demonstrates how to use the DS1603L sensor component
 * for basic distance measurements and level calculations.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ds1603l_sensor.h"

static const char *TAG = "EXAMPLE";

void app_main(void)
{
    ESP_LOGI(TAG, "DS1603L Basic Reading Example");

    // Configure sensor
    ds1603l_config_t config = DS1603L_DEFAULT_CONFIG();
    config.tx_pin = 27;           // ESP32 TX to sensor RX
    config.rx_pin = 22;           // ESP32 RX to sensor TX
    config.tank_height_mm = 1000; // 1-meter tank
    config.filter_size = 15;      // Larger filter for stability

    // Initialize sensor
    ds1603l_handle_t sensor;
    esp_err_t ret = ds1603l_init(&config, &sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize sensor: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Sensor initialized successfully");
    ESP_LOGI(TAG, "Starting measurements...");

    // Main measurement loop
    while (1)
    {
        uint16_t distance_mm;
        ds1603l_status_t status;
        bool is_ok, new_data;

        // Read filtered distance
        ret = ds1603l_read_filtered(sensor, &distance_mm);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read sensor: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Get sensor status and health
        ds1603l_get_status(sensor, &status);
        ds1603l_is_sensor_ok(sensor, &is_ok);
        ds1603l_is_new_data_available(sensor, &new_data);

        if (is_ok && distance_mm != 0xFFFF)
        {
            // Calculate level percentage
            uint8_t level_percent;
            ds1603l_calculate_level(sensor, distance_mm, &level_percent);

            // Get raw reading for comparison
            uint16_t raw_distance;
            ds1603l_get_raw_reading(sensor, &raw_distance);

            // Log results
            ESP_LOGI(TAG, "Distance: %d mm (raw: %d mm), Level: %d%%, Status: %d, New data: %s",
                     distance_mm, raw_distance, level_percent, status,
                     new_data ? "Yes" : "No");

            // Example: Alert if tank is nearly empty (< 10%)
            if (level_percent < 10)
            {
                ESP_LOGW(TAG, "⚠️  Tank level low: %d%%", level_percent);
            }

            // Example: Alert if tank is nearly full (> 90%)
            if (level_percent > 90)
            {
                ESP_LOGW(TAG, "⚠️  Tank level high: %d%%", level_percent);
            }
        }
        else
        {
            // Sensor not working properly
            const char *status_str;
            switch (status)
            {
            case DS1603L_NO_SENSOR_DETECTED:
                status_str = "No sensor detected";
                break;
            case DS1603L_READING_CHECKSUM_FAIL:
                status_str = "Checksum failed";
                break;
            default:
                status_str = "Unknown error";
                break;
            }
            ESP_LOGW(TAG, "Sensor issue: %s", status_str);
        }

        // Print detailed status every 10 readings
        static int count = 0;
        if (++count >= 10)
        {
            count = 0;
            ds1603l_print_status(sensor);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Read every second
    }

    // Cleanup (unreachable in this example)
    ds1603l_deinit(sensor);
}