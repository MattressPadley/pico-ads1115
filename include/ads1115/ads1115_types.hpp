/**
 * @file ads1115_types.hpp
 * @brief Type definitions and enumerations for the ADS1115 ADC library
 *
 * This header contains all the type definitions, enumerations, and structures
 * used by the ADS1115 library. It defines the configuration options, error
 * codes, data structures for readings and device configuration, and callback
 * function types.
 *
 */

#pragma once

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <cstdint>

/**
 * @namespace ADS1115
 * @brief Contains all classes, functions, and types for the ADS1115 ADC library
 */
namespace ADS1115 {

/**
 * @enum ADCChannel
 * @brief ADC input channel selection enumeration
 *
 * Defines the available ADC input channels including single-ended and
 * differential channel configurations. The ADS1115 supports 4 single-ended
 * inputs (A0-A3) and 4 differential input combinations.
 */
enum class ADCChannel : uint8_t {
  A0 = 0,               ///< Single-ended channel A0
  A1 = 1,               ///< Single-ended channel A1
  A2 = 2,               ///< Single-ended channel A2
  A3 = 3,               ///< Single-ended channel A3
  DIFFERENTIAL_0_1 = 4, ///< Differential channel A0 - A1
  DIFFERENTIAL_0_3 = 5, ///< Differential channel A0 - A3
  DIFFERENTIAL_1_3 = 6, ///< Differential channel A1 - A3
  DIFFERENTIAL_2_3 = 7  ///< Differential channel A2 - A3
};

/**
 * @enum GainAmplifier
 * @brief Programmable gain amplifier (PGA) settings
 *
 * Defines the available gain settings for the internal programmable gain
 * amplifier. Each setting corresponds to a specific full-scale voltage range.
 * Higher gain settings provide better resolution for smaller voltage signals
 * but reduce the maximum measurable voltage.
 */
enum class GainAmplifier : uint8_t {
  GAIN_TWOTHIRDS = 0x00, ///< 2/3x gain, ±6.144V range, 187.5μV/LSB
  GAIN_ONE = 0x01,       ///< 1x gain, ±4.096V range, 125μV/LSB
  GAIN_TWO = 0x02,       ///< 2x gain, ±2.048V range, 62.5μV/LSB (default)
  GAIN_FOUR = 0x03,      ///< 4x gain, ±1.024V range, 31.25μV/LSB
  GAIN_EIGHT = 0x04,     ///< 8x gain, ±0.512V range, 15.625μV/LSB
  GAIN_SIXTEEN = 0x05    ///< 16x gain, ±0.256V range, 7.8125μV/LSB
};

/**
 * @enum DataRate
 * @brief ADC data rate (samples per second) settings
 *
 * Defines the available conversion rates for the ADC. Lower data rates
 * provide better noise performance and higher effective resolution,
 * while higher data rates allow faster sampling at the cost of slightly
 * higher noise.
 */
enum class DataRate : uint8_t {
  SPS_8 = 0x00,   ///< 8 samples per second (~125ms conversion time)
  SPS_16 = 0x01,  ///< 16 samples per second (~62.5ms conversion time)
  SPS_32 = 0x02,  ///< 32 samples per second (~31.25ms conversion time)
  SPS_64 = 0x03,  ///< 64 samples per second (~15.6ms conversion time)
  SPS_128 = 0x04, ///< 128 samples per second (~7.8ms conversion time, default)
  SPS_250 = 0x05, ///< 250 samples per second (~4ms conversion time)
  SPS_475 = 0x06, ///< 475 samples per second (~2.1ms conversion time)
  SPS_860 = 0x07  ///< 860 samples per second (~1.16ms conversion time)
};

/**
 * @enum OperatingMode
 * @brief ADC operating mode selection
 *
 * Defines the two operating modes of the ADS1115: single-shot mode where
 * each conversion must be explicitly requested, and continuous mode where
 * conversions happen automatically at the configured data rate.
 */
enum class OperatingMode : uint8_t {
  CONTINUOUS = 0x00, ///< Continuous conversion mode (auto-repeat)
  SINGLE_SHOT = 0x01 ///< Single-shot conversion mode (default, power saving)
};

/**
 * @enum ComparatorMode
 * @brief Comparator operation mode
 *
 * Defines how the built-in comparator operates when comparing ADC readings
 * to threshold values.
 */
enum class ComparatorMode : uint8_t {
  TRADITIONAL =
      0x00, ///< Traditional comparator (alert when reading > high threshold)
  WINDOW =
      0x01 ///< Window comparator (alert when reading outside low-high range)
};

/**
 * @enum ComparatorPolarity
 * @brief Alert pin polarity setting
 *
 * Defines the electrical polarity of the alert pin when an alert condition
 * is detected by the comparator.
 */
enum class ComparatorPolarity : uint8_t {
  ACTIVE_LOW = 0x00, ///< Alert pin pulls low when active (default)
  ACTIVE_HIGH = 0x01 ///< Alert pin drives high when active
};

/**
 * @enum ComparatorLatch
 * @brief Comparator latching behavior
 *
 * Defines whether the alert condition latches (remains active until cleared)
 * or automatically clears when the condition is no longer met.
 */
enum class ComparatorLatch : uint8_t {
  NON_LATCHING =
      0x00,       ///< Alert clears automatically when condition ends (default)
  LATCHING = 0x01 ///< Alert remains active until explicitly cleared
};

/**
 * @enum ComparatorQueue
 * @brief Comparator queue setting
 *
 * Defines how many consecutive readings must exceed the threshold before
 * the alert is asserted. This helps reduce false alerts due to noise.
 */
enum class ComparatorQueue : uint8_t {
  ASSERT_AFTER_ONE = 0x00,  ///< Assert alert after one threshold crossing
  ASSERT_AFTER_TWO = 0x01,  ///< Assert alert after two consecutive crossings
  ASSERT_AFTER_FOUR = 0x02, ///< Assert alert after four consecutive crossings
  DISABLE = 0x03            ///< Disable comparator functionality (default)
};

/**
 * @enum Error
 * @brief Error codes returned by library functions
 *
 * Defines all possible error conditions that can be returned by library
 * functions. Success is indicated by Error::SUCCESS (0), while all error
 * conditions have negative values.
 */
enum class Error : int8_t {
  SUCCESS = 0,              ///< Operation completed successfully
  I2C_ERROR = -1,           ///< I2C communication error
  DEVICE_NOT_FOUND = -2,    ///< ADS1115 device not responding
  INVALID_PARAMETER = -3,   ///< Invalid function parameter provided
  TIMEOUT = -4,             ///< Operation timed out
  NOT_INITIALIZED = -5,     ///< Device not initialized (call begin() first)
  CONFIGURATION_ERROR = -6, ///< Device configuration error
  CONVERSION_ERROR = -7     ///< ADC conversion error
};

/**
 * @struct ADCConfig
 * @brief ADC configuration structure
 *
 * Contains all configurable parameters for the ADS1115 ADC including
 * gain, data rate, operating mode, and comparator settings. Used to
 * configure the device behavior.
 */
struct ADCConfig {
  GainAmplifier gain =
      GainAmplifier::GAIN_TWO; ///< PGA gain setting (±2.048V range default)
  DataRate data_rate = DataRate::SPS_128; ///< Conversion rate (128 SPS default)
  OperatingMode mode =
      OperatingMode::SINGLE_SHOT; ///< Operating mode (single-shot default)
  ComparatorMode comparator_mode =
      ComparatorMode::TRADITIONAL; ///< Comparator mode
  ComparatorPolarity comparator_polarity =
      ComparatorPolarity::ACTIVE_LOW; ///< Alert polarity
  ComparatorLatch comparator_latch =
      ComparatorLatch::NON_LATCHING; ///< Latch behavior
  ComparatorQueue comparator_queue =
      ComparatorQueue::DISABLE; ///< Queue setting

  /**
   * @brief Default constructor with standard settings
   */
  ADCConfig() = default;
};

/**
 * @struct ADCReading
 * @brief ADC reading result structure
 *
 * Contains the complete result of an ADC reading including the raw value,
 * converted voltage, channel information, timestamp, and validity flag.
 * This structure provides all information about a single ADC measurement.
 */
struct ADCReading {
  int16_t raw_value = 0; ///< Raw 16-bit ADC value (-32768 to 32767)
  float voltage = 0.0f;  ///< Converted voltage value in volts
  ADCChannel channel = ADCChannel::A0; ///< Channel that was read
  uint32_t timestamp = 0;              ///< System timestamp in microseconds
  bool valid = false;                  ///< Reading validity flag

  /**
   * @brief Default constructor - creates invalid reading
   */
  ADCReading() = default;

  /**
   * @brief Constructor with all parameters
   *
   * @param raw Raw ADC value
   * @param volt Converted voltage
   * @param ch Channel that was read
   * @param time Timestamp (0 = current time)
   */
  ADCReading(int16_t raw, float volt, ADCChannel ch, uint32_t time = 0)
      : raw_value(raw), voltage(volt), channel(ch), timestamp(time),
        valid(true) {}
};

/**
 * @struct DeviceStatus
 * @brief Device status information structure
 *
 * Provides comprehensive status information about the ADS1115 device
 * including readiness, conversion status, alert conditions, and
 * performance statistics.
 */
struct DeviceStatus {
  bool device_ready = false; ///< Device is initialized and ready for operation
  bool conversion_ready = false; ///< Current conversion is complete
  bool comparator_alert = false; ///< Comparator alert condition is active
  bool overflow = false;         ///< ADC overflow condition detected
  uint16_t conversion_count = 0; ///< Total number of conversions performed

  /**
   * @brief Default constructor - all status flags false
   */
  DeviceStatus() = default;
};

/**
 * @struct ThresholdConfig
 * @brief Comparator threshold configuration structure
 *
 * Defines the voltage thresholds for the comparator functionality.
 * Used in both traditional and window comparator modes.
 */
struct ThresholdConfig {
  float high_threshold = 3.0f; ///< High threshold voltage (upper limit)
  float low_threshold = 1.0f;  ///< Low threshold voltage (lower limit)
  bool enabled = false;        ///< Enable threshold monitoring

  /**
   * @brief Default constructor with example thresholds
   */
  ThresholdConfig() = default;

  /**
   * @brief Constructor with specified thresholds
   *
   * @param high High threshold voltage
   * @param low Low threshold voltage
   * @param en Enable threshold monitoring
   */
  ThresholdConfig(float high, float low, bool en = true)
      : high_threshold(high), low_threshold(low), enabled(en) {}
};

/**
 * @name Callback Function Types
 * @brief Function pointer types for event callbacks
 * @{
 */

/**
 * @typedef ConversionCallback
 * @brief Callback function type for conversion completion events
 *
 * Called when an ADC conversion completes. Provides the reading result.
 *
 * @param reading The completed ADC reading
 */
using ConversionCallback = void (*)(const ADCReading &reading);

/**
 * @typedef AlertCallback
 * @brief Callback function type for alert events
 *
 * Called when a comparator alert condition occurs or clears.
 *
 * @param channel The channel that triggered the alert
 * @param alert_active True if alert is active, false if cleared
 */
using AlertCallback = void (*)(ADCChannel channel, bool alert_active);

/**
 * @typedef ErrorCallback
 * @brief Callback function type for error events
 *
 * Called when an error condition occurs during operation.
 *
 * @param error The error code
 * @param message Optional descriptive message (may be nullptr)
 */
using ErrorCallback = void (*)(Error error, const char *message);

/** @} */

/**
 * @name Voltage Conversion Helper Functions
 * @brief Constexpr functions for voltage calculations
 * @{
 */

/**
 * @brief Get full-scale voltage range for gain setting
 *
 * Returns the maximum positive voltage that can be measured
 * with the specified gain setting.
 *
 * @param gain Gain amplifier setting
 * @return Full-scale voltage range in volts
 */
constexpr float getVoltageRange(GainAmplifier gain) {
  switch (gain) {
  case GainAmplifier::GAIN_TWOTHIRDS:
    return 6.144f; // ±6.144V
  case GainAmplifier::GAIN_ONE:
    return 4.096f; // ±4.096V
  case GainAmplifier::GAIN_TWO:
    return 2.048f; // ±2.048V
  case GainAmplifier::GAIN_FOUR:
    return 1.024f; // ±1.024V
  case GainAmplifier::GAIN_EIGHT:
    return 0.512f; // ±0.512V
  case GainAmplifier::GAIN_SIXTEEN:
    return 0.256f; // ±0.256V
  default:
    return 2.048f; // Default case
  }
}

/**
 * @brief Get voltage resolution per LSB for gain setting
 *
 * Returns the voltage represented by one LSB (least significant bit)
 * of the ADC reading for the specified gain setting.
 *
 * @param gain Gain amplifier setting
 * @return Voltage per bit in volts
 */
constexpr float getVoltagePerBit(GainAmplifier gain) {
  return getVoltageRange(gain) / 32767.0f; // 15-bit signed resolution
}

/** @} */

/**
 * @name Data Rate Conversion Helper Functions
 * @brief Functions for converting between data rate settings and frequencies
 * @{
 */

/**
 * @brief Convert data rate setting to frequency in Hz
 *
 * @param rate Data rate setting
 * @return Sampling frequency in Hz
 */
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
    return 128; // Default rate
  }
}

/**
 * @brief Get conversion time in milliseconds for data rate
 *
 * Returns the expected time for one conversion to complete
 * including a safety margin.
 *
 * @param rate Data rate setting
 * @return Conversion time in milliseconds
 */
constexpr uint32_t getConversionTimeMs(DataRate rate) {
  return 1000 / getDataRateHz(rate) + 1; // Add 1ms for safety margin
}

/** @} */

/**
 * @name Channel Validation Helper Functions
 * @brief Functions for validating and categorizing ADC channels
 * @{
 */

/**
 * @brief Check if channel is valid
 *
 * @param channel Channel to validate
 * @return true if channel is valid
 */
constexpr bool isValidChannel(ADCChannel channel) {
  return static_cast<uint8_t>(channel) <=
         static_cast<uint8_t>(ADCChannel::DIFFERENTIAL_2_3);
}

/**
 * @brief Check if channel is single-ended
 *
 * @param channel Channel to check
 * @return true if channel is single-ended (A0-A3)
 */
constexpr bool isSingleEndedChannel(ADCChannel channel) {
  return static_cast<uint8_t>(channel) <= static_cast<uint8_t>(ADCChannel::A3);
}

/**
 * @brief Check if channel is differential
 *
 * @param channel Channel to check
 * @return true if channel is differential
 */
constexpr bool isDifferentialChannel(ADCChannel channel) {
  return static_cast<uint8_t>(channel) >=
         static_cast<uint8_t>(ADCChannel::DIFFERENTIAL_0_1);
}

/** @} */

/**
 * @name Voltage Calculation Helper Functions
 * @brief Functions for converting between raw values and voltages
 * @{
 */

/**
 * @brief Convert raw ADC value to voltage
 *
 * @param raw_value Raw 16-bit ADC value
 * @param gain Current gain amplifier setting
 * @return Voltage value in volts
 */
constexpr float rawToVoltage(int16_t raw_value, GainAmplifier gain) {
  return static_cast<float>(raw_value) * getVoltagePerBit(gain);
}

/**
 * @brief Convert voltage to raw ADC value
 *
 * @param voltage Voltage to convert
 * @param gain Current gain amplifier setting
 * @return Raw ADC value (clamped to valid range)
 */
constexpr int16_t voltageToRaw(float voltage, GainAmplifier gain) {
  float raw_float = voltage / getVoltagePerBit(gain);
  if (raw_float > 32767.0f)
    return 32767;
  if (raw_float < -32768.0f)
    return -32768;
  return static_cast<int16_t>(raw_float);
}

/** @} */

/**
 * @struct DeviceConfig
 * @brief Comprehensive device configuration structure
 *
 * Contains all hardware and software configuration options for the ADS1115
 * device including I2C setup, pin assignments, timing parameters, and
 * default ADC settings. Used for complete device initialization.
 */
struct DeviceConfig {
  // I2C Hardware Configuration
  i2c_inst_t *i2c_instance = nullptr; ///< I2C instance pointer (i2c0 or i2c1)
  uint8_t device_address = 0x48;      ///< I2C device address (0x48-0x4B)
  uint32_t i2c_baudrate = 100000; ///< I2C clock speed in Hz (typically 100kHz)
  uint sda_pin = PICO_DEFAULT_I2C_SDA_PIN; ///< SDA pin number
  uint scl_pin = PICO_DEFAULT_I2C_SCL_PIN; ///< SCL pin number
  bool enable_pullups = true; ///< Enable internal pull-up resistors

  // Alert/Interrupt Configuration
  uint alert_pin = 255;       ///< Alert pin number (255 = disabled)
  bool alert_enabled = false; ///< Enable alert functionality

  // Power Management
  uint16_t power_on_delay_ms = 25; ///< Power-on stabilization delay
  uint16_t reset_delay_ms = 10;    ///< Reset stabilization delay

  // Default ADC Configuration
  ADCConfig default_adc_config = ADCConfig(); ///< Default ADC settings to apply

  // I2C Communication Settings
  uint8_t i2c_timeout_ms = 100; ///< I2C operation timeout
  uint8_t max_retries = 3;      ///< Maximum I2C retry attempts

  // Validation flags
  bool auto_init_i2c = false;       ///< Automatically initialize I2C hardware
  bool validate_connections = true; ///< Validate I2C connections during begin()

  /**
   * @brief Default constructor with standard settings
   */
  DeviceConfig() = default;

  /**
   * @brief Constructor with basic I2C settings
   *
   * @param i2c_inst I2C instance pointer
   * @param addr Device I2C address
   * @param sda SDA pin number
   * @param scl SCL pin number
   */
  DeviceConfig(i2c_inst_t *i2c_inst, uint8_t addr = 0x48,
               uint sda = PICO_DEFAULT_I2C_SDA_PIN,
               uint scl = PICO_DEFAULT_I2C_SCL_PIN)
      : i2c_instance(i2c_inst), device_address(addr), sda_pin(sda),
        scl_pin(scl) {}

  /**
   * @brief Constructor with full I2C configuration
   *
   * @param i2c_inst I2C instance pointer
   * @param addr Device I2C address
   * @param baudrate I2C clock speed in Hz
   * @param sda SDA pin number
   * @param scl SCL pin number
   * @param alert Alert pin number (255 = disabled)
   */
  DeviceConfig(i2c_inst_t *i2c_inst, uint8_t addr, uint32_t baudrate, uint sda,
               uint scl, uint alert = 255)
      : i2c_instance(i2c_inst), device_address(addr), i2c_baudrate(baudrate),
        sda_pin(sda), scl_pin(scl), alert_pin(alert) {}
};

/**
 * @name String Conversion Helper Functions
 * @brief Functions for converting enums to human-readable strings
 * @{
 */

/**
 * @brief Convert ADC channel to descriptive string
 * @param channel Channel to convert
 * @return Pointer to static string describing the channel
 */
const char *channelToString(ADCChannel channel);

/**
 * @brief Convert gain setting to descriptive string
 * @param gain Gain setting to convert
 * @return Pointer to static string describing the gain
 */
const char *gainToString(GainAmplifier gain);

/**
 * @brief Convert data rate to descriptive string
 * @param rate Data rate to convert
 * @return Pointer to static string describing the data rate
 */
const char *dataRateToString(DataRate rate);

/**
 * @brief Convert error code to descriptive string
 * @param error Error code to convert
 * @return Pointer to static string describing the error
 */
const char *errorToString(Error error);

/** @} */

} // namespace ADS1115
