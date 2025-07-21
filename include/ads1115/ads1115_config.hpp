/**
 * @file ads1115_config.hpp
 * @brief Configuration definitions and validation for the ADS1115 library
 *
 * This header contains compile-time configuration options, default values,
 * and validation macros for the ADS1115 library. It allows customization
 * of default behavior without modifying the core library code.
 *
 */

#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "ads1115_types.hpp"

/**
 * @namespace ADS1115
 * @brief Contains all classes, functions, and types for the ADS1115 ADC library
 */
namespace ADS1115 {

/**
 * @name Default I2C Configuration
 * @brief Default I2C interface settings
 *
 * These macros define the default I2C configuration used when not explicitly
 * specified. They can be overridden by defining them before including this
 * header.
 * @{
 */

/** @brief Default I2C instance (i2c0 or i2c1) */
#ifndef ADS1115_DEFAULT_I2C_INSTANCE
#define ADS1115_DEFAULT_I2C_INSTANCE i2c_default
#endif

/** @brief Default I2C clock frequency in Hz */
#ifndef ADS1115_DEFAULT_I2C_BAUDRATE
#define ADS1115_DEFAULT_I2C_BAUDRATE 100000
#endif

/** @brief Default SDA pin number */
#ifndef ADS1115_DEFAULT_SDA_PIN
#define ADS1115_DEFAULT_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#endif

/** @brief Default SCL pin number */
#ifndef ADS1115_DEFAULT_SCL_PIN
#define ADS1115_DEFAULT_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#endif

/** @brief Default pull-up resistor enable setting */
#ifndef ADS1115_DEFAULT_ENABLE_PULLUPS
#define ADS1115_DEFAULT_ENABLE_PULLUPS true
#endif

/** @} */

/**
 * @name Default Device Address
 * @brief Default I2C device address
 * @{
 */

/** @brief Default ADS1115 I2C address (ADDR pin to GND) */
#ifndef ADS1115_DEFAULT_DEVICE_ADDRESS
#define ADS1115_DEFAULT_DEVICE_ADDRESS 0x48
#endif

/** @} */

/**
 * @name Default Alert Pin Configuration
 * @brief Default alert/interrupt pin settings
 * @{
 */

/** @brief Default alert pin number (255 = disabled) */
#ifndef ADS1115_DEFAULT_ALERT_PIN
#define ADS1115_DEFAULT_ALERT_PIN 255 // Disabled
#endif

/** @brief Default alert functionality enable state */
#ifndef ADS1115_DEFAULT_ALERT_ENABLED
#define ADS1115_DEFAULT_ALERT_ENABLED false
#endif

/** @} */

/**
 * @name Default Power Management
 * @brief Default timing and power management settings
 * @{
 */

/** @brief Default power-on stabilization delay in milliseconds */
#ifndef ADS1115_DEFAULT_POWER_ON_DELAY_MS
#define ADS1115_DEFAULT_POWER_ON_DELAY_MS 25
#endif

/** @brief Default reset stabilization delay in milliseconds */
#ifndef ADS1115_DEFAULT_RESET_DELAY_MS
#define ADS1115_DEFAULT_RESET_DELAY_MS 10
#endif

/** @} */

/**
 * @name Default I2C Communication Settings
 * @brief Default I2C communication parameters
 * @{
 */

/** @brief Default I2C operation timeout in milliseconds */
#ifndef ADS1115_DEFAULT_I2C_TIMEOUT_MS
#define ADS1115_DEFAULT_I2C_TIMEOUT_MS 100
#endif

/** @brief Default maximum I2C retry attempts */
#ifndef ADS1115_DEFAULT_MAX_RETRIES
#define ADS1115_DEFAULT_MAX_RETRIES 3
#endif

/** @} */

/**
 * @name Default Validation Settings
 * @brief Default validation and initialization behavior
 * @{
 */

/** @brief Default automatic I2C initialization setting */
#ifndef ADS1115_DEFAULT_AUTO_INIT_I2C
#define ADS1115_DEFAULT_AUTO_INIT_I2C false
#endif

/** @brief Default connection validation setting */
#ifndef ADS1115_DEFAULT_VALIDATE_CONNECTIONS
#define ADS1115_DEFAULT_VALIDATE_CONNECTIONS true
#endif

/** @} */

/**
 * @name Configuration Validation Macros
 * @brief Macros for validating configuration parameters
 *
 * These macros provide compile-time and runtime validation of configuration
 * parameters to ensure they are within valid ranges.
 * @{
 */

/**
 * @brief Validate I2C instance pointer
 * @param inst I2C instance to validate
 * @return true if instance is valid (i2c0 or i2c1)
 */
#define ADS1115_VALIDATE_I2C_INSTANCE(inst) ((inst) == i2c0 || (inst) == i2c1)

/**
 * @brief Validate I2C device address
 * @param addr Address to validate
 * @return true if address is in valid range (0x48-0x4B)
 */
#define ADS1115_VALIDATE_I2C_ADDRESS(addr) ((addr) >= 0x48 && (addr) <= 0x4B)

/**
 * @brief Validate I2C baudrate
 * @param rate Baudrate to validate
 * @return true if baudrate is in reasonable range (10kHz-1MHz)
 */
#define ADS1115_VALIDATE_I2C_BAUDRATE(rate)                                    \
  ((rate) >= 10000 && (rate) <= 1000000)

/**
 * @brief Validate GPIO pin number
 * @param pin Pin number to validate
 * @return true if pin is valid GPIO pin (not 255)
 */
#define ADS1115_VALIDATE_GPIO_PIN(pin) ((pin) < 256 && (pin) != 255)

/**
 * @brief Validate alert pin number
 * @param pin Pin number to validate
 * @return true if pin is valid (255 = disabled is allowed)
 */
#define ADS1115_VALIDATE_ALERT_PIN(pin) ((pin) < 256) // 255 = disabled

/** @} */

/**
 * @name Configuration Helper Functions
 * @brief Functions for creating and validating configurations
 * @{
 */

/**
 * @brief Create a DeviceConfig with all default values
 *
 * Creates a DeviceConfig structure populated with all the default values
 * defined by the configuration macros. This provides a convenient starting
 * point for creating custom configurations.
 *
 * @return DeviceConfig structure with default values
 */
inline DeviceConfig createDefaultDeviceConfig() {
  DeviceConfig config;
  config.i2c_instance = ADS1115_DEFAULT_I2C_INSTANCE;
  config.device_address = ADS1115_DEFAULT_DEVICE_ADDRESS;
  config.i2c_baudrate = ADS1115_DEFAULT_I2C_BAUDRATE;
  config.sda_pin = ADS1115_DEFAULT_SDA_PIN;
  config.scl_pin = ADS1115_DEFAULT_SCL_PIN;
  config.enable_pullups = ADS1115_DEFAULT_ENABLE_PULLUPS;
  config.alert_pin = ADS1115_DEFAULT_ALERT_PIN;
  config.alert_enabled = ADS1115_DEFAULT_ALERT_ENABLED;
  config.power_on_delay_ms = ADS1115_DEFAULT_POWER_ON_DELAY_MS;
  config.reset_delay_ms = ADS1115_DEFAULT_RESET_DELAY_MS;
  config.i2c_timeout_ms = ADS1115_DEFAULT_I2C_TIMEOUT_MS;
  config.max_retries = ADS1115_DEFAULT_MAX_RETRIES;
  config.auto_init_i2c = ADS1115_DEFAULT_AUTO_INIT_I2C;
  config.validate_connections = ADS1115_DEFAULT_VALIDATE_CONNECTIONS;
  return config;
}

/**
 * @brief Validate a DeviceConfig structure
 *
 * Performs comprehensive validation of all parameters in a DeviceConfig
 * structure to ensure they are within valid ranges and logically consistent.
 *
 * @param config DeviceConfig structure to validate
 * @return true if configuration is valid, false otherwise
 *
 * @note This function checks:
 *       - I2C instance is valid (i2c0 or i2c1)
 *       - Device address is in valid range (0x48-0x4B)
 *       - Baudrate is reasonable (10kHz-1MHz)
 *       - GPIO pins are valid
 *       - SDA and SCL pins are different
 *       - Alert pin is valid (255 = disabled is allowed)
 */
inline bool validateDeviceConfig(const DeviceConfig &config) {
  if (!ADS1115_VALIDATE_I2C_INSTANCE(config.i2c_instance)) {
    return false;
  }
  if (!ADS1115_VALIDATE_I2C_ADDRESS(config.device_address)) {
    return false;
  }
  if (!ADS1115_VALIDATE_I2C_BAUDRATE(config.i2c_baudrate)) {
    return false;
  }
  if (!ADS1115_VALIDATE_GPIO_PIN(config.sda_pin)) {
    return false;
  }
  if (!ADS1115_VALIDATE_GPIO_PIN(config.scl_pin)) {
    return false;
  }
  if (!ADS1115_VALIDATE_ALERT_PIN(config.alert_pin)) {
    return false;
  }
  if (config.sda_pin == config.scl_pin) {
    return false; // SDA and SCL cannot be the same pin
  }
  return true;
}

/** @} */

} // namespace ADS1115
