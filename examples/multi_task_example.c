/**
 * @file multi_task_example.c
 * @brief Advanced DS1603L sensor usage with multiple tasks
 *
 * This example demonstrates thread-safe usage of the DS1603L sensor
 * component across multiple FreeRTOS tasks.
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "ds1603l_sensor.h"

static const char *TAG = "MULTI_TASK";

// Shared sensor handle
static ds1603l_handle_t g_sensor = NULL;

// Message queue for inter-task communication
static QueueHandle_t sensor_data_queue = NULL;

// Data structure for sensor readings
typedef struct
{
    uint16_t distance_mm;
    uint8_t level_percent;
    ds1603l_status_t status;
    uint32_t timestamp;
    bool valid;
} sensor_reading_t;

/**
 * @brief Sensor reading task
 *
 * Continuously reads the sensor and sends data to other tasks
 */
void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");

    while (1)
    {
        sensor_reading_t reading = {0};
        uint16_t distance_mm;
        bool new_data;

        // Read sensor
        esp_err_t ret = ds1603l_read_filtered(g_sensor, &distance_mm);
        if (ret == ESP_OK)
        {
            reading.distance_mm = distance_mm;
            reading.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;

            // Get status and check for new data
            ds1603l_get_status(g_sensor, &reading.status);
            ds1603l_is_new_data_available(g_sensor, &new_data);

            if (distance_mm != 0xFFFF && new_data)
            {
                // Calculate level
                ds1603l_calculate_level(g_sensor, distance_mm, &reading.level_percent);
                reading.valid = true;

                // Send to queue (non-blocking)
                if (xQueueSend(sensor_data_queue, &reading, 0) != pdTRUE)
                {
                    ESP_LOGW(TAG, "Queue full, dropping reading");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Read every 100ms
    }
}

/**
 * @brief Data processing task
 *
 * Processes sensor data and performs calculations
 */
void processing_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Processing task started");

    sensor_reading_t reading;
    uint32_t total_readings = 0;
    uint32_t valid_readings = 0;
    uint32_t sum_distance = 0;

    while (1)
    {
        // Wait for new sensor data
        if (xQueueReceive(sensor_data_queue, &reading, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            total_readings++;

            if (reading.valid)
            {
                valid_readings++;
                sum_distance += reading.distance_mm;

                ESP_LOGI(TAG, "📊 Reading #%lu: %d mm, %d%%, Status: %d",
                         total_readings, reading.distance_mm,
                         reading.level_percent, reading.status);

                // Calculate average distance every 10 valid readings
                if (valid_readings % 10 == 0)
                {
                    uint32_t avg_distance = sum_distance / valid_readings;
                    ESP_LOGI(TAG, "📈 Average distance: %lu mm (%lu valid readings)",
                             avg_distance, valid_readings);
                }

                // Trigger alerts based on level
                if (reading.level_percent < 5)
                {
                    ESP_LOGE(TAG, "🚨 CRITICAL: Tank nearly empty (%d%%)", reading.level_percent);
                }
                else if (reading.level_percent < 15)
                {
                    ESP_LOGW(TAG, "⚠️  WARNING: Tank level low (%d%%)", reading.level_percent);
                }
                else if (reading.level_percent > 95)
                {
                    ESP_LOGW(TAG, "⚠️  WARNING: Tank nearly full (%d%%)", reading.level_percent);
                }
            }
            else
            {
                ESP_LOGW(TAG, "❌ Invalid reading, status: %d", reading.status);
            }
        }
        else
        {
            ESP_LOGD(TAG, "No new sensor data received");
        }
    }
}

/**
 * @brief Monitoring task
 *
 * Monitors sensor health and provides periodic status updates
 */
void monitoring_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Monitoring task started");

    uint32_t last_status_time = 0;
    uint32_t error_count = 0;

    while (1)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        bool is_ok;
        ds1603l_status_t status;

        // Check sensor health
        esp_err_t ret = ds1603l_is_sensor_ok(g_sensor, &is_ok);
        if (ret == ESP_OK)
        {
            ds1603l_get_status(g_sensor, &status);

            if (!is_ok)
            {
                error_count++;
                ESP_LOGW(TAG, "🔧 Sensor health check failed (count: %lu), status: %d",
                         error_count, status);
            }
            else if (error_count > 0)
            {
                ESP_LOGI(TAG, "✅ Sensor recovered after %lu errors", error_count);
                error_count = 0;
            }
        }

        // Print detailed status every 30 seconds
        if (current_time - last_status_time > 30000)
        {
            last_status_time = current_time;
            ESP_LOGI(TAG, "🔍 Periodic status check:");
            ds1603l_print_status(g_sensor);

            // Print task stack watermarks
            ESP_LOGI(TAG, "📋 Task stack usage:");
            ESP_LOGI(TAG, "  Sensor task: %d bytes free",
                     uxTaskGetStackHighWaterMark(NULL));
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Monitor every 5 seconds
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "DS1603L Multi-Task Example");

    // Configure sensor
    ds1603l_config_t config = DS1603L_DEFAULT_CONFIG();
    config.tx_pin = 27;
    config.rx_pin = 22;
    config.tank_height_mm = 1500; // 1.5-meter tank
    config.filter_size = 20;      // Large filter for stability

    // Initialize sensor
    esp_err_t ret = ds1603l_init(&config, &g_sensor);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize sensor: %s", esp_err_to_name(ret));
        return;
    }

    // Create message queue
    sensor_data_queue = xQueueCreate(10, sizeof(sensor_reading_t));
    if (!sensor_data_queue)
    {
        ESP_LOGE(TAG, "Failed to create sensor data queue");
        ds1603l_deinit(g_sensor);
        return;
    }

    ESP_LOGI(TAG, "Starting tasks...");

    // Create tasks
    xTaskCreatePinnedToCore(
        sensor_task,
        "sensor_task",
        4096,
        NULL,
        5, // High priority for sensor reading
        NULL,
        1 // Core 1
    );

    xTaskCreatePinnedToCore(
        processing_task,
        "processing_task",
        4096,
        NULL,
        3, // Medium priority
        NULL,
        0 // Core 0
    );

    xTaskCreatePinnedToCore(
        monitoring_task,
        "monitoring_task",
        3072,
        NULL,
        2, // Low priority
        NULL,
        0 // Core 0
    );

    ESP_LOGI(TAG, "All tasks started successfully");

    // Main task can do other work or just monitor
    while (1)
    {
        // Print memory usage every minute
        ESP_LOGI(TAG, "💾 Free heap: %lu bytes", esp_get_free_heap_size());
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}