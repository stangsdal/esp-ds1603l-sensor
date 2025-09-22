# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2025-09-22

### Added
- Initial release of ESP-IDF DS1603L sensor component
- Thread-safe UART communication with DS1603L ultrasonic sensor
- Moving average filtering for stable readings
- Tank level percentage calculation
- Comprehensive error handling and logging
- Example applications demonstrating usage
- Full API documentation
- CMake build system integration

### Features
- Configurable UART pins and baud rate
- Automatic packet validation with checksum verification
- Configurable moving average filter (1-255 samples)
- Sensor health monitoring and timeout detection
- Debug logging support
- Clean C API for easy integration

### Documentation
- Complete README with installation and usage instructions
- API documentation in header files
- Working examples for common use cases
- Troubleshooting guide