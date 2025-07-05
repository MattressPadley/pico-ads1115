#pragma once

#include "pico/stdlib.h"
#include "hardware/i2c.h"

namespace ADS1115 {

// Default I2C Configuration
#ifndef ADS1115_DEFAULT_I2C_INSTANCE
#define ADS1115_DEFAULT_I2C_INSTANCE i2c_default
#endif

#ifndef ADS1115_DEFAULT_I2C_BAUDRATE
#define ADS1115_DEFAULT_I2C_BAUDRATE 100000
#endif

#ifndef ADS1115_DEFAULT_SDA_PIN
#define ADS1115_DEFAULT_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#endif

#ifndef ADS1115_DEFAULT_SCL_PIN
#define ADS1115_DEFAULT_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN
#endif

#ifndef ADS1115_DEFAULT_ENABLE_PULLUPS
#define ADS1115_DEFAULT_ENABLE_PULLUPS true
#endif

// Default Device Address
#ifndef ADS1115_DEFAULT_DEVICE_ADDRESS
#define ADS1115_DEFAULT_DEVICE_ADDRESS 0x48
#endif

// Default Alert Pin Configuration
#ifndef ADS1115_DEFAULT_ALERT_PIN
#define ADS1115_DEFAULT_ALERT_PIN 255  // Disabled
#endif

#ifndef ADS1115_DEFAULT_ALERT_ENABLED
#define ADS1115_DEFAULT_ALERT_ENABLED false
#endif

// Default Power Management
#ifndef ADS1115_DEFAULT_POWER_ON_DELAY_MS
#define ADS1115_DEFAULT_POWER_ON_DELAY_MS 25
#endif

#ifndef ADS1115_DEFAULT_CONVERSION_DELAY_MS
#define ADS1115_DEFAULT_CONVERSION_DELAY_MS 10
#endif

// Default I2C Communication Settings
#ifndef ADS1115_DEFAULT_I2C_TIMEOUT_MS
#define ADS1115_DEFAULT_I2C_TIMEOUT_MS 100
#endif

#ifndef ADS1115_DEFAULT_MAX_RETRIES
#define ADS1115_DEFAULT_MAX_RETRIES 3
#endif

// Default Validation Settings
#ifndef ADS1115_DEFAULT_AUTO_INIT_I2C
#define ADS1115_DEFAULT_AUTO_INIT_I2C false
#endif

#ifndef ADS1115_DEFAULT_VALIDATE_CONNECTIONS
#define ADS1115_DEFAULT_VALIDATE_CONNECTIONS true
#endif

// Configuration validation macros
#define ADS1115_VALIDATE_I2C_INSTANCE(inst) \
    ((inst) == i2c0 || (inst) == i2c1)

#define ADS1115_VALIDATE_I2C_ADDRESS(addr) \
    ((addr) >= 0x48 && (addr) <= 0x4B)

#define ADS1115_VALIDATE_I2C_BAUDRATE(rate) \
    ((rate) >= 10000 && (rate) <= 1000000)

#define ADS1115_VALIDATE_GPIO_PIN(pin) \
    ((pin) < 256 && (pin) != 255)

#define ADS1115_VALIDATE_ALERT_PIN(pin) \
    ((pin) < 256)  // 255 = disabled

// Helper function to create default DeviceConfig
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
    config.conversion_delay_ms = ADS1115_DEFAULT_CONVERSION_DELAY_MS;
    config.i2c_timeout_ms = ADS1115_DEFAULT_I2C_TIMEOUT_MS;
    config.max_retries = ADS1115_DEFAULT_MAX_RETRIES;
    config.auto_init_i2c = ADS1115_DEFAULT_AUTO_INIT_I2C;
    config.validate_connections = ADS1115_DEFAULT_VALIDATE_CONNECTIONS;
    return config;
}

// Helper function to validate DeviceConfig
inline bool validateDeviceConfig(const DeviceConfig& config) {
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
        return false;  // SDA and SCL cannot be the same pin
    }
    return true;
}

} // namespace ADS1115