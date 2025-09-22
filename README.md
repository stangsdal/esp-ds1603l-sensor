# ESP-IDF DS1603L Sensor Component

[![ESP-IDF Compatible](https://img.shields.io/badge/ESP--IDF-v5.0+-blue.svg)](https://github.com/espressif/esp-idf)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A reusable ESP-IDF component for the DS1603L ultrasonic distance sensor, designed for tank level monitoring and distance measurement applications.

## Overview

This component provides a complete, production-ready driver for the DS1603L ultrasonic distance sensor with the following features:

- **Thread-safe operation** with mutex protection for multi-task environments
- **Moving average filtering** for stable and reliable readings
- **Automatic sensor health monitoring** and timeout detection
- **Level percentage calculation** for tank monitoring applications
- **Clean C API** for easy integration into any ESP-IDF project
- **Configurable parameters** for different use cases and environments
- **Comprehensive error handling** with ESP-IDF standard error codes

## Features

- ✅ UART communication with configurable pins and baud rate
- ✅ Automatic packet validation with checksum verification
- ✅ Configurable moving average filter (1-255 samples)
- ✅ Tank level percentage calculation
- ✅ Sensor health monitoring and timeout detection
- ✅ Thread-safe API with mutex protection
- ✅ Comprehensive error handling
- ✅ Debug logging support

## Hardware Requirements

- ESP32 or compatible ESP-IDF supported chip
- DS1603L ultrasonic distance sensor
- UART connection (TX/RX pins)

## Wiring

| DS1603L Pin | ESP32 Pin | Description                      |
| ----------- | --------- | -------------------------------- |
| VCC         | 3.3V/5V   | Power supply                     |
| GND         | GND       | Ground                           |
| TX          | GPIO (RX) | Sensor transmit to ESP receive   |
| RX          | GPIO (TX) | Sensor receive from ESP transmit |

## Installation

### Method 1: Git Submodule (Recommended)

Add this component as a git submodule to your ESP-IDF project:

```bash
# From your ESP-IDF project root
git submodule add https://github.com/stangsdal/esp-ds1603l-sensor.git components/ds1603l_sensor
git submodule update --init --recursive
```

### Method 2: Direct Download

1. Download or clone this repository:
```bash
git clone https://github.com/stangsdal/esp-ds1603l-sensor.git
```

2. Copy to your project's components directory:
```bash
cp -r esp-ds1603l-sensor /path/to/your/project/components/ds1603l_sensor
```

### Method 3: Global ESP-IDF Installation

Install globally for all ESP-IDF projects:

```bash
git clone https://github.com/stangsdal/esp-ds1603l-sensor.git
cp -r esp-ds1603l-sensor $IDF_PATH/components/ds1603l_sensor
```

## Usage

### Basic Example

```c
#include "ds1603l_sensor.h"

void app_main(void)
{
    // Configure sensor
    ds1603l_config_t config = DS1603L_DEFAULT_CONFIG();
    config.tx_pin = 27;  // ESP32 TX to sensor RX
    config.rx_pin = 22;  // ESP32 RX to sensor TX
    config.tank_height_mm = 1000;  // 1-meter tank

    // Initialize sensor
    ds1603l_handle_t sensor;
    esp_err_t ret = ds1603l_init(&config, &sensor);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize sensor: %s", esp_err_to_name(ret));
        return;
    }

    while (1) {
        uint16_t distance_mm;

        // Read filtered distance
        ret = ds1603l_read_filtered(sensor, &distance_mm);
        if (ret == ESP_OK && distance_mm != 0xFFFF) {
            // Calculate level percentage
            uint8_t level_percent;
            ds1603l_calculate_level(sensor, distance_mm, &level_percent);

            printf("Distance: %d mm, Level: %d%%\\n", distance_mm, level_percent);
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Read every second
    }

    // Cleanup (if needed)
    ds1603l_deinit(sensor);
}
```

### Advanced Usage with Status Monitoring

```c
#include "ds1603l_sensor.h"

void sensor_task(void *pvParameters)
{
    ds1603l_handle_t sensor = (ds1603l_handle_t)pvParameters;

    while (1) {
        uint16_t distance_mm;
        ds1603l_status_t status;
        bool is_ok, new_data;

        // Read sensor
        esp_err_t ret = ds1603l_read_filtered(sensor, &distance_mm);
        if (ret != ESP_OK) {
            ESP_LOGE("SENSOR", "Failed to read sensor: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Check sensor health
        ds1603l_is_sensor_ok(sensor, &is_ok);
        ds1603l_get_status(sensor, &status);
        ds1603l_is_new_data_available(sensor, &new_data);

        if (is_ok && new_data) {
            uint8_t level_percent;
            ds1603l_calculate_level(sensor, distance_mm, &level_percent);

            ESP_LOGI("SENSOR", "Distance: %d mm, Level: %d%%, Status: %d",
                     distance_mm, level_percent, status);
        } else if (!is_ok) {
            ESP_LOGW("SENSOR", "Sensor not OK, status: %d", status);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## Configuration

### Default Configuration

The `DS1603L_DEFAULT_CONFIG()` macro provides sensible defaults:

```c
{
    .uart_num = UART_NUM_2,
    .tx_pin = 27,
    .rx_pin = 22,
    .baud_rate = 9600,
    .uart_buffer_size = 256,
    .read_timeout_ms = 5000,
    .sensor_timeout_ms = 10000,
    .filter_size = 10,
    .tank_height_mm = 400
}
```

### Customizing Configuration

```c
ds1603l_config_t config = DS1603L_DEFAULT_CONFIG();

// Customize specific parameters
config.uart_num = UART_NUM_1;          // Use UART1 instead
config.tx_pin = 17;                    // Different TX pin
config.rx_pin = 16;                    // Different RX pin
config.filter_size = 20;               // Larger filter for more stability
config.tank_height_mm = 2000;          // 2-meter tank
config.read_timeout_ms = 3000;         // Faster reading rate
```

## API Reference

### Initialization Functions

- `ds1603l_init()` - Initialize sensor with configuration
- `ds1603l_deinit()` - Cleanup and deinitialize sensor

### Reading Functions

- `ds1603l_read()` - Read raw sensor data
- `ds1603l_read_filtered()` - Read filtered sensor data
- `ds1603l_get_raw_reading()` - Get last raw reading without new read

### Status Functions

- `ds1603l_get_status()` - Get current sensor status
- `ds1603l_is_sensor_ok()` - Check if sensor is working properly
- `ds1603l_is_new_data_available()` - Check for new data since last read
- `ds1603l_print_status()` - Print status for debugging

### Utility Functions

- `ds1603l_calculate_level()` - Calculate level percentage from distance
- `ds1603l_set_tank_height()` - Update tank height configuration

## Error Handling

All functions return `esp_err_t` codes:

- `ESP_OK` - Operation successful
- `ESP_ERR_INVALID_ARG` - Invalid parameter
- `ESP_ERR_NO_MEM` - Memory allocation failed
- `ESP_ERR_TIMEOUT` - Mutex timeout or sensor timeout
- Other ESP-IDF error codes for UART operations

## Sensor Status Codes

- `DS1603L_NO_SENSOR_DETECTED` - Sensor not responding
- `DS1603L_READING_SUCCESS` - Valid data received
- `DS1603L_READING_CHECKSUM_FAIL` - Data received but checksum invalid

## Performance Characteristics

- **Memory Usage**: ~200 bytes + filter buffer (filter_size \* 2 bytes)
- **CPU Usage**: Minimal, only during UART data processing
- **Response Time**: ~100ms typical reading time
- **Accuracy**: ±1mm (sensor dependent)
- **Thread Safety**: Full mutex protection for concurrent access

## Troubleshooting

### Common Issues

1. **No sensor detected**: Check wiring and power supply
2. **Checksum failures**: Check for electrical interference or baud rate mismatch
3. **Inconsistent readings**: Increase filter size or check sensor mounting
4. **Memory allocation failures**: Reduce filter size or increase available heap

### Debug Logging

Enable debug logging in menuconfig:

- Component config → Log output → Default log verbosity: Debug

Or set log level programmatically:

```c
esp_log_level_set("DS1603L", ESP_LOG_DEBUG);
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

### Development Setup

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Coding Standards

- Follow ESP-IDF coding conventions
- Add appropriate logging for debugging
- Include error handling for all operations
- Update documentation for API changes

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built for ESP-IDF framework by Espressif Systems
- Designed for use with DS1603L ultrasonic sensors
- Inspired by the need for reliable tank level monitoring solutions

## Support

- Create an [issue](https://github.com/stangsdal/esp-ds1603l-sensor/issues) for bug reports or feature requests
- Check the [examples](examples/) directory for usage patterns
- Review the API documentation in the header files

---

**Made with ❤️ for the ESP-IDF community**

## License

This component is released under the same license as the parent project.

## Credits

- Original DS1603L library by Wouter van Marle (wouter@cityhydronics.hk, 2018)
- ESP-IDF adaptation by Peter Stangsdal (2025)
