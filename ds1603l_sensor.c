/**
 * @file ds1603l_sensor.c
 * @brief DS1603L Ultrasonic Distance Sensor Driver Implementation
 *
 * Originally written by Wouter van Marle, wouter@cityhydronics.hk, 2018
 * Adapted for ESP-IDF by Peter Stangsdal, 2025
 */

#include "ds1603l_sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "DS1603L";

/**
 * @brief DS1603L sensor instance structure
 */
struct ds1603l_sensor
{
    ds1603l_config_t config;    /**< Sensor configuration */
    uart_port_t uart_num;       /**< UART port number */
    ds1603l_status_t status;    /**< Current sensor status */
    uint32_t serial_data;       /**< 4-byte rolling buffer for UART data */
    uint16_t last_reading;      /**< Latest raw sensor reading */
    uint32_t last_reading_time; /**< Timestamp of last successful reading */
    uint32_t last_read_time;    /**< Timestamp of last read attempt */
    bool new_data_available;    /**< Flag indicating new data since last read */
    bool initialized;           /**< Initialization flag */

    // Moving average filter
    uint16_t *filter_buffer; /**< Filter buffer */
    uint8_t filter_index;    /**< Current filter index */
    bool filter_filled;      /**< Filter buffer filled flag */

    // Mutex for thread safety
    SemaphoreHandle_t mutex; /**< Mutex for thread-safe access */
};

/**
 * @brief Calculate checksum for DS1603L data packet
 */
static uint8_t calculate_checksum(uint32_t data)
{
    uint8_t checksum = 0xFF;         // Start byte
    checksum += (data >> 16) & 0xFF; // Data high byte
    checksum += (data >> 8) & 0xFF;  // Data low byte
    return checksum;
}

/**
 * @brief Add reading to moving average filter
 */
static uint16_t filter_add_reading(ds1603l_handle_t handle, uint16_t value)
{
    if (!handle || !handle->filter_buffer)
    {
        return value;
    }

    handle->filter_buffer[handle->filter_index] = value;
    handle->filter_index = (handle->filter_index + 1) % handle->config.filter_size;

    if (handle->filter_index == 0)
    {
        handle->filter_filled = true;
    }

    // Calculate average
    uint32_t sum = 0;
    uint8_t count = handle->filter_filled ? handle->config.filter_size : handle->filter_index;

    for (uint8_t i = 0; i < count; i++)
    {
        sum += handle->filter_buffer[i];
    }

    return count > 0 ? (sum / count) : value;
}

/**
 * @brief Process incoming UART data
 */
static esp_err_t process_uart_data(ds1603l_handle_t handle)
{
    uint8_t data[256];
    int len = uart_read_bytes(handle->uart_num, data, sizeof(data), 0);

    if (len <= 0)
    {
        return ESP_OK; // No data available
    }

    // Process each byte
    for (int i = 0; i < len; i++)
    {
        uint8_t byte = data[i];
        handle->serial_data = (handle->serial_data << 8) | byte;

        // Check for complete packet (start byte = 0xFF)
        if ((handle->serial_data >> 24) == 0xFF)
        {
            uint8_t expected_checksum = calculate_checksum(handle->serial_data);
            uint8_t received_checksum = handle->serial_data & 0xFF;

            if (expected_checksum == received_checksum)
            {
                // Valid packet
                handle->last_reading = (handle->serial_data >> 8) & 0xFFFF;
                handle->status = DS1603L_READING_SUCCESS;
                handle->last_reading_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                handle->new_data_available = true;
                handle->initialized = true;
                ESP_LOGD(TAG, "Valid reading: %d mm", handle->last_reading);
            }
            else
            {
                // Checksum failed
                handle->status = DS1603L_READING_CHECKSUM_FAIL;
                handle->last_reading = (handle->serial_data >> 8) & 0xFFFF;
                handle->last_reading_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                handle->new_data_available = true;
                handle->initialized = true;
                ESP_LOGW(TAG, "Checksum failed: expected 0x%02X, got 0x%02X",
                         expected_checksum, received_checksum);
            }
        }
    }

    return ESP_OK;
}

esp_err_t ds1603l_init(const ds1603l_config_t *config, ds1603l_handle_t *handle)
{
    if (!config || !handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing DS1603L sensor");

    // Allocate sensor handle
    ds1603l_handle_t sensor = malloc(sizeof(struct ds1603l_sensor));
    if (!sensor)
    {
        ESP_LOGE(TAG, "Failed to allocate sensor handle");
        return ESP_ERR_NO_MEM;
    }

    memset(sensor, 0, sizeof(struct ds1603l_sensor));
    memcpy(&sensor->config, config, sizeof(ds1603l_config_t));
    sensor->uart_num = config->uart_num;

    // Allocate filter buffer
    sensor->filter_buffer = malloc(config->filter_size * sizeof(uint16_t));
    if (!sensor->filter_buffer)
    {
        ESP_LOGE(TAG, "Failed to allocate filter buffer");
        free(sensor);
        return ESP_ERR_NO_MEM;
    }
    memset(sensor->filter_buffer, 0, config->filter_size * sizeof(uint16_t));

    // Create mutex
    sensor->mutex = xSemaphoreCreateMutex();
    if (!sensor->mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(sensor->filter_buffer);
        free(sensor);
        return ESP_ERR_NO_MEM;
    }

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(config->uart_num, config->uart_buffer_size, 0, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        vSemaphoreDelete(sensor->mutex);
        free(sensor->filter_buffer);
        free(sensor);
        return ret;
    }

    ret = uart_param_config(config->uart_num, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_num);
        vSemaphoreDelete(sensor->mutex);
        free(sensor->filter_buffer);
        free(sensor);
        return ret;
    }

    ret = uart_set_pin(config->uart_num, config->tx_pin, config->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(config->uart_num);
        vSemaphoreDelete(sensor->mutex);
        free(sensor->filter_buffer);
        free(sensor);
        return ret;
    }

    // Initialize sensor state
    sensor->status = DS1603L_NO_SENSOR_DETECTED;
    sensor->last_reading_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    sensor->last_read_time = 0;
    sensor->new_data_available = false;
    sensor->initialized = false;

    *handle = sensor;
    ESP_LOGI(TAG, "DS1603L sensor initialized successfully");

    return ESP_OK;
}

esp_err_t ds1603l_deinit(ds1603l_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Deinitializing DS1603L sensor");

    // Delete UART driver
    uart_driver_delete(handle->uart_num);

    // Delete mutex
    if (handle->mutex)
    {
        vSemaphoreDelete(handle->mutex);
    }

    // Free memory
    if (handle->filter_buffer)
    {
        free(handle->filter_buffer);
    }
    free(handle);

    ESP_LOGI(TAG, "DS1603L sensor deinitialized");
    return ESP_OK;
}

esp_err_t ds1603l_read(ds1603l_handle_t handle, uint16_t *distance_mm)
{
    if (!handle || !distance_mm)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Check if it's time to read (respect timeout)
    if (current_time - handle->last_read_time > handle->config.read_timeout_ms)
    {
        handle->last_read_time = current_time;

        // Trigger sensor reading by sending a command
        uint8_t cmd = 0x55;
        uart_write_bytes(handle->uart_num, &cmd, 1);

        // Wait a bit for response
        vTaskDelay(pdMS_TO_TICKS(100));

        // Process any incoming data
        process_uart_data(handle);
    }

    // Check for sensor timeout
    if (current_time - handle->last_reading_time > handle->config.sensor_timeout_ms)
    {
        handle->status = DS1603L_NO_SENSOR_DETECTED;
        ESP_LOGW(TAG, "Sensor timeout detected");
    }

    // Return current reading
    if (handle->status == DS1603L_NO_SENSOR_DETECTED)
    {
        *distance_mm = 0xFFFF; // Invalid reading indicator
    }
    else
    {
        *distance_mm = handle->last_reading;
    }

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_read_filtered(ds1603l_handle_t handle, uint16_t *distance_mm)
{
    if (!handle || !distance_mm)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t raw_reading;
    esp_err_t ret = ds1603l_read(handle, &raw_reading);
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (raw_reading != 0xFFFF)
    {
        *distance_mm = filter_add_reading(handle, raw_reading);
    }
    else
    {
        *distance_mm = raw_reading;
    }

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_get_status(ds1603l_handle_t handle, ds1603l_status_t *status)
{
    if (!handle || !status)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    *status = handle->status;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_calculate_level(ds1603l_handle_t handle, uint16_t distance_mm, uint8_t *level_percent)
{
    if (!handle || !level_percent)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (distance_mm == 0xFFFF)
    {
        *level_percent = 0;
        return ESP_OK;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    uint32_t percent = (distance_mm * 100) / handle->config.tank_height_mm;
    if (percent > 100)
    {
        percent = 100;
    }

    *level_percent = (uint8_t)percent;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_is_sensor_ok(ds1603l_handle_t handle, bool *is_ok)
{
    if (!handle || !is_ok)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    *is_ok = handle->initialized &&
             (handle->status == DS1603L_READING_SUCCESS ||
              handle->status == DS1603L_READING_CHECKSUM_FAIL);

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_is_new_data_available(ds1603l_handle_t handle, bool *new_data)
{
    if (!handle || !new_data)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    *new_data = handle->new_data_available;
    handle->new_data_available = false; // Clear flag after checking

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_get_raw_reading(ds1603l_handle_t handle, uint16_t *raw_distance_mm)
{
    if (!handle || !raw_distance_mm)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    if (handle->status == DS1603L_NO_SENSOR_DETECTED)
    {
        *raw_distance_mm = 0xFFFF;
    }
    else
    {
        *raw_distance_mm = handle->last_reading;
    }

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_set_tank_height(ds1603l_handle_t handle, uint16_t tank_height_mm)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    handle->config.tank_height_mm = tank_height_mm;

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}

esp_err_t ds1603l_print_status(ds1603l_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    const char *status_str;
    switch (handle->status)
    {
    case DS1603L_NO_SENSOR_DETECTED:
        status_str = "No sensor detected";
        break;
    case DS1603L_READING_SUCCESS:
        status_str = "Reading success";
        break;
    case DS1603L_READING_CHECKSUM_FAIL:
        status_str = "Checksum failed";
        break;
    default:
        status_str = "Unknown status";
        break;
    }

    ESP_LOGI(TAG, "DS1603L Status: %s, Last reading: %d mm, Initialized: %s",
             status_str, handle->last_reading, handle->initialized ? "Yes" : "No");

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}