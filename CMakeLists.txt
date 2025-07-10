cmake_minimum_required(VERSION 3.13)

# Export compile commands for IDEs and tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Include the Pico SDK only if not already available
if(NOT COMMAND pico_sdk_init)
  # Check if PICO_SDK_PATH is set, otherwise download to build folder
  if(NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH ${CMAKE_CURRENT_BINARY_DIR}/pico-sdk)
    if(NOT EXISTS ${PICO_SDK_PATH})
      message(STATUS "Downloading Pico SDK to ${PICO_SDK_PATH}")
      execute_process(
                COMMAND git clone --depth 1 --branch 2.1.1 https://github.com/raspberrypi/pico-sdk.git ${PICO_SDK_PATH}
                RESULT_VARIABLE GIT_RESULT
            )
      if(NOT GIT_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to download Pico SDK")
      endif()
      execute_process(
                COMMAND git submodule update --init --recursive
                WORKING_DIRECTORY ${PICO_SDK_PATH}
                RESULT_VARIABLE GIT_RESULT
            )
      if(NOT GIT_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to initialize Pico SDK submodules")
      endif()
    endif()
    include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
  else()
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
  endif()
endif()

# Only configure project if we're the top-level project
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  # Project configuration
  project(pico_ads1115 C CXX ASM)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_CXX_STANDARD 17)

  # Initialize the Pico SDK
  pico_sdk_init()
endif()

# Library source files
set(ADS1115_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/ads1115.cpp
)

# Library headers
set(ADS1115_HEADERS
    ${CMAKE_CURRENT_LIST_DIR}/include/ads1115/ads1115.hpp
    ${CMAKE_CURRENT_LIST_DIR}/include/ads1115/ads1115_registers.hpp
    ${CMAKE_CURRENT_LIST_DIR}/include/ads1115/ads1115_types.hpp
)

# Create the ADS1115 library
add_library(ads1115 STATIC ${ADS1115_SOURCES})

# Include directories for the library
target_include_directories(ads1115 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

# Link Pico SDK libraries to our library
target_link_libraries(ads1115
    pico_stdlib
    hardware_i2c
    hardware_gpio
)

# Set compile options for the library
target_compile_options(ads1115 PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
)

# Enable USB output for printf debugging (optional)
# Uncomment the following lines to enable USB stdio
# pico_enable_stdio_usb(ads1115 1)
# pico_enable_stdio_uart(ads1115 0)

# Only build examples if we're the top-level project
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  # Basic ADC example
  add_executable(basic_adc examples/basic_adc/basic_adc.cpp)

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

  # Create map/bin/hex/uf2 files for the example
  pico_add_extra_outputs(basic_adc)
endif()

# Installation rules (optional)
install(TARGETS ads1115
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp"
)

# Package configuration (optional)
# set(ADS1115_VERSION "0.1.0")
# configure_file(
#     "${CMAKE_CURRENT_SOURCE_DIR}/ads1115-config.cmake.in"
#     "${CMAKE_CURRENT_BINARY_DIR}/ads1115-config.cmake"
#     @ONLY
# )

# Add compile definitions
target_compile_definitions(ads1115 PRIVATE
    ADS1115_VERSION_MAJOR=0
    ADS1115_VERSION_MINOR=1
    ADS1115_VERSION_PATCH=0
)

# Add version information to the example
target_compile_definitions(basic_adc PRIVATE
    ADS1115_VERSION_MAJOR=0
    ADS1115_VERSION_MINOR=1
    ADS1115_VERSION_PATCH=0
)
