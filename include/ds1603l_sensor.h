/**
 * @file ds1603l_sensor.h
 * @brief DS1603L Ultrasonic Distance Sensor Driver for ESP-IDF
 *
 * This component provides a driver for the DS1603L ultrasonic distance sensor
 * with moving average filtering and level calculation capabilities.
 *
 * Originally written by Wouter van Marle, wouter@cityhydronics.hk, 2018
 * Adapted for ESP-IDF by Peter Stangsdal, 2025
 */

#ifndef DS1603L_SENSOR_H
#define DS1603L_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief DS1603L sensor status codes
     */
    typedef enum
    {
        DS1603L_NO_SENSOR_DETECTED = 0,   /**< Sensor not detected */
        DS1603L_READING_SUCCESS = 1,      /**< Latest reading successful */
        DS1603L_READING_CHECKSUM_FAIL = 2 /**< Latest reading failed checksum */
    } ds1603l_status_t;

    /**
     * @brief DS1603L sensor configuration structure
     */
    typedef struct
    {
        uart_port_t uart_num;       /**< UART port number */
        int tx_pin;                 /**< TX pin number */
        int rx_pin;                 /**< RX pin number */
        uint32_t baud_rate;         /**< UART baud rate (typically 9600) */
        size_t uart_buffer_size;    /**< UART buffer size */
        uint32_t read_timeout_ms;   /**< Sensor read timeout in milliseconds */
        uint32_t sensor_timeout_ms; /**< Sensor disconnect timeout in milliseconds */
        uint8_t filter_size;        /**< Moving average filter size */
        uint16_t tank_height_mm;    /**< Total tank height for percentage calculation */
    } ds1603l_config_t;

    /**
     * @brief DS1603L sensor handle
     */
    typedef struct ds1603l_sensor *ds1603l_handle_t;

/**
 * @brief Default configuration for DS1603L sensor
 */
#define DS1603L_DEFAULT_CONFIG() { \
    .uart_num = UART_NUM_2,        \
    .tx_pin = 27,                  \
    .rx_pin = 22,                  \
    .baud_rate = 9600,             \
    .uart_buffer_size = 256,       \
    .read_timeout_ms = 5000,       \
    .sensor_timeout_ms = 10000,    \
    .filter_size = 10,             \
    .tank_height_mm = 400}

    /**
     * @brief Initialize DS1603L sensor
     *
     * @param config Pointer to sensor configuration
     * @param handle Pointer to store sensor handle
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_init(const ds1603l_config_t *config, ds1603l_handle_t *handle);

    /**
     * @brief Deinitialize DS1603L sensor
     *
     * @param handle Sensor handle
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_deinit(ds1603l_handle_t handle);

    /**
     * @brief Read sensor data
     *
     * @param handle Sensor handle
     * @param distance_mm Pointer to store distance reading in mm
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_read(ds1603l_handle_t handle, uint16_t *distance_mm);

    /**
     * @brief Read filtered sensor data (with moving average)
     *
     * @param handle Sensor handle
     * @param distance_mm Pointer to store filtered distance reading in mm
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_read_filtered(ds1603l_handle_t handle, uint16_t *distance_mm);

    /**
     * @brief Get sensor status
     *
     * @param handle Sensor handle
     * @param status Pointer to store sensor status
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_get_status(ds1603l_handle_t handle, ds1603l_status_t *status);

    /**
     * @brief Calculate level percentage from distance
     *
     * @param handle Sensor handle
     * @param distance_mm Distance reading in mm
     * @param level_percent Pointer to store level percentage (0-100)
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_calculate_level(ds1603l_handle_t handle, uint16_t distance_mm, uint8_t *level_percent);

    /**
     * @brief Check if sensor is working properly
     *
     * @param handle Sensor handle
     * @param is_ok Pointer to store sensor health status
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_is_sensor_ok(ds1603l_handle_t handle, bool *is_ok);

    /**
     * @brief Check if new data is available since last read
     *
     * @param handle Sensor handle
     * @param new_data Pointer to store new data flag
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_is_new_data_available(ds1603l_handle_t handle, bool *new_data);

    /**
     * @brief Get raw (unfiltered) sensor reading
     *
     * @param handle Sensor handle
     * @param raw_distance_mm Pointer to store raw distance reading in mm
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_get_raw_reading(ds1603l_handle_t handle, uint16_t *raw_distance_mm);

    /**
     * @brief Set tank height for level percentage calculation
     *
     * @param handle Sensor handle
     * @param tank_height_mm Tank height in millimeters
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_set_tank_height(ds1603l_handle_t handle, uint16_t tank_height_mm);

    /**
     * @brief Print sensor status for debugging
     *
     * @param handle Sensor handle
     * @return esp_err_t ESP_OK on success
     */
    esp_err_t ds1603l_print_status(ds1603l_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // DS1603L_SENSOR_H