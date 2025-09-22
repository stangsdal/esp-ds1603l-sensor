# DS1603L Sensor Component CMakeLists.txt
# ESP-IDF component for DS1603L ultrasonic distance sensor

set(COMPONENT_SRCS 
    "ds1603l_sensor.c"
)

set(COMPONENT_ADD_INCLUDEDIRS 
    "include"
)

set(COMPONENT_REQUIRES 
    driver
    esp_common
    freertos
)

set(COMPONENT_PRIV_REQUIRES
    esp_timer
)

register_component()

# Component version information
set_property(TARGET ${COMPONENT_LIB} PROPERTY VERSION "1.0.0")
set_property(TARGET ${COMPONENT_LIB} PROPERTY SOVERSION "1")

# Add component description
set_property(TARGET ${COMPONENT_LIB} PROPERTY DESCRIPTION 
    "DS1603L ultrasonic distance sensor driver for ESP-IDF")

# Export header for other components
target_include_directories(${COMPONENT_LIB} INTERFACE
    $<BUILD_INTERFACE:${COMPONENT_DIR}/include>
    $<INSTALL_INTERFACE:include>
)