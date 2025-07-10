#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdint>

namespace ADS1115 {

// ADC channel enumeration
enum class ADCChannel : uint8_t {
  A0 = 0,
  A1 = 1,
  A2 = 2,
  A3 = 3,
  DIFFERENTIAL_0_1 = 4,
  DIFFERENTIAL_0_3 = 5,
  DIFFERENTIAL_1_3 = 6,
  DIFFERENTIAL_2_3 = 7
};

// Programmable gain amplifier settings
enum class GainAmplifier : uint8_t {
  GAIN_TWOTHIRDS = 0x00, // ±6.144V range
  GAIN_ONE = 0x01,       // ±4.096V range
  GAIN_TWO = 0x02,       // ±2.048V range (default)
  GAIN_FOUR = 0x03,      // ±1.024V range
  GAIN_EIGHT = 0x04,     // ±0.512V range
  GAIN_SIXTEEN = 0x05    // ±0.256V range
};

// Data rate settings
enum class DataRate : uint8_t {
  SPS_8 = 0x00,   // 8 samples per second
  SPS_16 = 0x01,  // 16 samples per second
  SPS_32 = 0x02,  // 32 samples per second
  SPS_64 = 0x03,  // 64 samples per second
  SPS_128 = 0x04, // 128 samples per second (default)
  SPS_250 = 0x05, // 250 samples per second
  SPS_475 = 0x06, // 475 samples per second
  SPS_860 = 0x07  // 860 samples per second
};

// Operating mode
enum class OperatingMode : uint8_t {
  CONTINUOUS = 0x00, // Continuous conversion mode
  SINGLE_SHOT = 0x01 // Single-shot conversion mode (default)
};

// Comparator mode
enum class ComparatorMode : uint8_t {
  TRADITIONAL = 0x00, // Traditional comparator
  WINDOW = 0x01       // Window comparator
};

// Comparator polarity
enum class ComparatorPolarity : uint8_t {
  ACTIVE_LOW = 0x00, // Active low (default)
  ACTIVE_HIGH = 0x01 // Active high
};

// Comparator latching
enum class ComparatorLatch : uint8_t {
  NON_LATCHING = 0x00, // Non-latching (default)
  LATCHING = 0x01      // Latching
};

// Comparator queue
enum class ComparatorQueue : uint8_t {
  ASSERT_AFTER_ONE = 0x00,  // Assert after one conversion
  ASSERT_AFTER_TWO = 0x01,  // Assert after two conversions
  ASSERT_AFTER_FOUR = 0x02, // Assert after four conversions
  DISABLE = 0x03            // Disable comparator (default)
};

// Error codes
enum class Error : int8_t {
  SUCCESS = 0,
  I2C_ERROR = -1,
  DEVICE_NOT_FOUND = -2,
  INVALID_PARAMETER = -3,
  TIMEOUT = -4,
  NOT_INITIALIZED = -5,
  CONFIGURATION_ERROR = -6,
  CONVERSION_ERROR = -7
};

// ADC configuration structure
struct ADCConfig {
  GainAmplifier gain = GainAmplifier::GAIN_TWO;    // ±2.048V range
  DataRate data_rate = DataRate::SPS_128;          // 128 samples per second
  OperatingMode mode = OperatingMode::SINGLE_SHOT; // Single-shot mode
  ComparatorMode comparator_mode = ComparatorMode::TRADITIONAL;
  ComparatorPolarity comparator_polarity = ComparatorPolarity::ACTIVE_LOW;
  ComparatorLatch comparator_latch = ComparatorLatch::NON_LATCHING;
  ComparatorQueue comparator_queue = ComparatorQueue::DISABLE;

  ADCConfig() = default;
};

// ADC reading structure
struct ADCReading {
  int16_t raw_value = 0; // Raw ADC value
  float voltage = 0.0f;  // Converted voltage
  ADCChannel channel = ADCChannel::A0;
  uint32_t timestamp = 0; // System timestamp
  bool valid = false;     // Reading validity flag

  ADCReading() = default;
  ADCReading(int16_t raw, float volt, ADCChannel ch, uint32_t time = 0)
      : raw_value(raw), voltage(volt), channel(ch), timestamp(time),
        valid(true) {}
};

// Device status information
struct DeviceStatus {
  bool device_ready = false;     // Device is ready for operation
  bool conversion_ready = false; // Conversion is complete
  bool comparator_alert = false; // Comparator alert active
  bool overflow = false;         // ADC overflow detected
  uint16_t conversion_count = 0; // Number of conversions performed

  DeviceStatus() = default;
};

// Threshold configuration for comparator
struct ThresholdConfig {
  float high_threshold = 3.0f; // High threshold voltage
  float low_threshold = 1.0f;  // Low threshold voltage
  bool enabled = false;        // Threshold monitoring enabled

  ThresholdConfig() = default;
  ThresholdConfig(float high, float low, bool en = true)
      : high_threshold(high), low_threshold(low), enabled(en) {}
};

// Callback function types
using ConversionCallback = void (*)(const ADCReading &reading);
using AlertCallback = void (*)(ADCChannel channel, bool alert_active);
using ErrorCallback = void (*)(Error error, const char *message);

// Helper functions for voltage conversion
constexpr float getVoltageRange(GainAmplifier gain) {
  switch (gain) {
  case GainAmplifier::GAIN_TWOTHIRDS:
    return 6.144f;
  case GainAmplifier::GAIN_ONE:
    return 4.096f;
  case GainAmplifier::GAIN_TWO:
    return 2.048f;
  case GainAmplifier::GAIN_FOUR:
    return 1.024f;
  case GainAmplifier::GAIN_EIGHT:
    return 0.512f;
  case GainAmplifier::GAIN_SIXTEEN:
    return 0.256f;
  default:
    return 2.048f;
  }
}

constexpr float getVoltagePerBit(GainAmplifier gain) {
  return getVoltageRange(gain) / 32767.0f; // 15-bit signed resolution
}

// Helper functions for data rate conversion
constexpr uint16_t getDataRateHz(DataRate rate) {
  switch (rate) {
  case DataRate::SPS_8:
    return 8;
  case DataRate::SPS_16:
    return 16;
  case DataRate::SPS_32:
    return 32;
  case DataRate::SPS_64:
    return 64;
  case DataRate::SPS_128:
    return 128;
  case DataRate::SPS_250:
    return 250;
  case DataRate::SPS_475:
    return 475;
  case DataRate::SPS_860:
    return 860;
  default:
    return 128;
  }
}

constexpr uint32_t getConversionTimeMs(DataRate rate) {
  return 1000 / getDataRateHz(rate) + 1; // Add 1ms for safety margin
}

// Channel validation and conversion helpers
constexpr bool isValidChannel(ADCChannel channel) {
  return static_cast<uint8_t>(channel) <=
         static_cast<uint8_t>(ADCChannel::DIFFERENTIAL_2_3);
}

constexpr bool isSingleEndedChannel(ADCChannel channel) {
  return static_cast<uint8_t>(channel) <= static_cast<uint8_t>(ADCChannel::A3);
}

constexpr bool isDifferentialChannel(ADCChannel channel) {
  return static_cast<uint8_t>(channel) >=
         static_cast<uint8_t>(ADCChannel::DIFFERENTIAL_0_1);
}

// Voltage calculation helper
constexpr float rawToVoltage(int16_t raw_value, GainAmplifier gain) {
  return static_cast<float>(raw_value) * getVoltagePerBit(gain);
}

constexpr int16_t voltageToRaw(float voltage, GainAmplifier gain) {
  float raw_float = voltage / getVoltagePerBit(gain);
  if (raw_float > 32767.0f)
    return 32767;
  if (raw_float < -32768.0f)
    return -32768;
  return static_cast<int16_t>(raw_float);
}

// Device configuration structure for external hardware setup
struct DeviceConfig {
  // I2C Hardware Configuration
  i2c_inst_t *i2c_instance = nullptr; // I2C instance (i2c0, i2c1)
  uint8_t device_address = 0x48; // I2C device address (default ADS1115 address)
  uint32_t i2c_baudrate = 100000;          // I2C clock speed in Hz
  uint sda_pin = PICO_DEFAULT_I2C_SDA_PIN; // SDA pin number
  uint scl_pin = PICO_DEFAULT_I2C_SCL_PIN; // SCL pin number
  bool enable_pullups = true;              // Enable internal pull-up resistors

  // Alert/Interrupt Configuration
  uint alert_pin = 255;       // Alert pin (255 = disabled)
  bool alert_enabled = false; // Enable alert functionality

  // Power Management
  uint16_t power_on_delay_ms = 25; // Power-on delay in milliseconds
  uint16_t reset_delay_ms = 10;    // Reset stabilization delay in milliseconds

  // Default ADC Configuration
  ADCConfig default_adc_config = ADCConfig(); // Default ADC settings

  // I2C Communication Settings
  uint8_t i2c_timeout_ms = 100; // I2C timeout in milliseconds
  uint8_t max_retries = 3;      // Maximum I2C retry attempts

  // Validation flags
  bool auto_init_i2c = false;       // Automatically initialize I2C hardware
  bool validate_connections = true; // Validate I2C connections on begin()

  DeviceConfig() = default;

  // Constructor with basic I2C settings
  DeviceConfig(i2c_inst_t *i2c_inst, uint8_t addr = 0x48,
               uint sda = PICO_DEFAULT_I2C_SDA_PIN,
               uint scl = PICO_DEFAULT_I2C_SCL_PIN)
      : i2c_instance(i2c_inst), device_address(addr), sda_pin(sda),
        scl_pin(scl) {}

  // Constructor with full configuration
  DeviceConfig(i2c_inst_t *i2c_inst, uint8_t addr, uint32_t baudrate, uint sda,
               uint scl, uint alert = 255)
      : i2c_instance(i2c_inst), device_address(addr), i2c_baudrate(baudrate),
        sda_pin(sda), scl_pin(scl), alert_pin(alert) {}
};

// String conversion helpers
const char *channelToString(ADCChannel channel);
const char *gainToString(GainAmplifier gain);
const char *dataRateToString(DataRate rate);
const char *errorToString(Error error);

} // namespace ADS1115
