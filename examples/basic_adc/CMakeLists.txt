cmake_minimum_required(VERSION 3.13)

# This example requires the parent project to be configured
if(NOT TARGET ads1115)
    message(FATAL_ERROR "This example requires the ads1115 library target to be available")
endif()

# Create the basic ADC example executable
add_executable(basic_adc basic_adc.cpp)

# Link libraries to the example
target_link_libraries(basic_adc 
    ads1115
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

# Enable USB and UART output for the example
pico_enable_stdio_usb(basic_adc 1)
pico_enable_stdio_uart(basic_adc 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(basic_adc)

# Add version information
target_compile_definitions(basic_adc PRIVATE
    ADS1115_VERSION_MAJOR=0
    ADS1115_VERSION_MINOR=1
    ADS1115_VERSION_PATCH=0
)