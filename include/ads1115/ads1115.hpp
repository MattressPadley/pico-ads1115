/**
 * @file ads1115.hpp
 * @brief Main header file for the ADS1115 ADC library for Raspberry Pi Pico
 *
 * This library provides a complete interface for the ADS1115 16-bit ADC with
 * programmable gain amplifier. It supports single-ended and differential
 * measurements, comparator functionality, continuous and single-shot conversion
 * modes, and configurable alert functionality.
 *
 *
 * @section Features
 * - 16-bit resolution analog-to-digital conversion
 * - 4 single-ended or 2 differential input channels
 * - Programmable gain amplifier (PGA) with 6 settings
 * - Configurable data rates from 8 to 860 samples per second
 * - Built-in voltage reference and oscillator
 * - Programmable comparator with alert/ready pin
 * - Single-shot and continuous conversion modes
 * - I2C interface with configurable address
 *
 * @section Usage
 * @code{.cpp}
 * #include "ads1115/ads1115.hpp"
 *
 * // Create device instance
 * ADS1115::ADS1115Device adc(i2c_default, 0x48);
 *
 * // Initialize
 * if (adc.begin() == ADS1115::Error::SUCCESS) {
 *     // Read channel A0
 *     ADS1115::ADCReading reading;
 *     if (adc.readChannel(ADS1115::ADCChannel::A0, reading) ==
 * ADS1115::Error::SUCCESS) { printf("Voltage: %.3fV\n", reading.voltage);
 *     }
 * }
 * @endcode
 */

#pragma once

#include "ads1115_config.hpp"
#include "ads1115_registers.hpp"
#include "ads1115_types.hpp"

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

/**
 * @namespace ADS1115
 * @brief Contains all classes, functions, and types for the ADS1115 ADC library
 */
namespace ADS1115 {

/**
 * @class ADS1115Device
 * @brief Main class for interfacing with the ADS1115 16-bit ADC
 *
 * This class provides a comprehensive interface for the ADS1115
 * analog-to-digital converter. It supports all features of the ADS1115
 * including single-ended and differential measurements, programmable gain
 * amplification, configurable data rates, and comparator functionality.
 *
 * The class handles all low-level I2C communication and provides high-level
 * functions for reading ADC values, configuring the device, and managing
 * continuous conversion modes.
 *
 * @note This class is thread-safe for read operations but write operations
 * should be synchronized if used in a multi-threaded environment.
 */
class ADS1115Device {
public:
  /**
   * @brief Construct ADS1115Device with basic I2C configuration
   *
   * Creates an ADS1115 device instance with the specified I2C interface and
   * device address. The alert pin is optional and can be disabled by setting it
   * to 255.
   *
   * @param i2c_instance Pointer to the I2C instance (i2c0 or i2c1)
   * @param device_address I2C address of the ADS1115 device (0x48-0x4B)
   * @param alert_pin GPIO pin number for alert functionality (255 = disabled)
   *
   * @note The I2C interface must be initialized before calling begin()
   *
   * @see begin()
   */
  explicit ADS1115Device(i2c_inst_t *i2c_instance,
                         uint8_t device_address = DEFAULT_I2C_ADDRESS,
                         uint alert_pin = 255);

  /**
   * @brief Construct ADS1115Device with comprehensive configuration
   *
   * Creates an ADS1115 device instance using a DeviceConfig structure that
   * includes all hardware configuration options including I2C setup, alert
   * configuration, and default ADC settings.
   *
   * @param config Complete device configuration structure
   *
   * @see DeviceConfig
   * @see begin(const DeviceConfig&)
   */
  explicit ADS1115Device(const DeviceConfig &config);

  /**
   * @brief Destructor
   *
   * Automatically stops continuous mode if active and disables the comparator
   * to ensure clean shutdown of the device.
   */
  ~ADS1115Device();

  /**
   * @name Device Initialization and Management
   * @brief Functions for initializing and managing the ADS1115 device
   * @{
   */

  /**
   * @brief Initialize the ADS1115 device with default settings
   *
   * Initializes the ADS1115 device by validating I2C communication, verifying
   * device presence, configuring default settings, and optionally setting up
   * alert functionality. Must be called before any other operations.
   *
   * @return Error::SUCCESS if initialization succeeds
   * @return Error::I2C_ERROR if I2C communication fails
   * @return Error::DEVICE_NOT_FOUND if device is not responding
   * @return Error::CONFIGURATION_ERROR if default configuration fails
   *
   * @note Waits for power-on delay before attempting communication
   *
   * @see Error
   */
  Error begin();

  /**
   * @brief Initialize the ADS1115 device with custom configuration
   *
   * Initializes the device using a comprehensive DeviceConfig structure that
   * includes I2C hardware setup, device addressing, alert configuration, and
   * default ADC settings.
   *
   * @param config Complete device configuration
   * @return Error::SUCCESS if initialization succeeds
   * @return Error::INVALID_PARAMETER if configuration is invalid
   * @return Error::I2C_ERROR if I2C setup or communication fails
   * @return Error::DEVICE_NOT_FOUND if device is not responding
   *
   * @see DeviceConfig
   */
  Error begin(const DeviceConfig &config);

  /**
   * @brief Reset the ADS1115 device to default state
   *
   * Performs a software reset by writing the reset configuration value to the
   * device and then reconfiguring with current settings.
   *
   * @return Error::SUCCESS if reset succeeds
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::I2C_ERROR if communication fails
   *
   * @note Device must be re-initialized after reset
   */
  Error reset();

  /**
   * @brief Check if the device is connected and responding
   *
   * Attempts to read the configuration register to verify I2C communication
   * and device presence.
   *
   * @return true if device is connected and responding
   * @return false if device is not responding or I2C error
   */
  bool isConnected();

  /**
   * @brief Get current device status information
   *
   * Returns a structure containing device status including readiness,
   * conversion status, alert status, and conversion count.
   *
   * @return DeviceStatus structure with current status
   *
   * @see DeviceStatus
   */
  DeviceStatus getStatus();

  /**
   * @brief Update the ADC configuration
   *
   * Updates the device configuration with new ADC settings including gain,
   * data rate, operating mode, and comparator settings.
   *
   * @param config New ADC configuration
   * @return Error::SUCCESS if configuration updated successfully
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ADCConfig
   */
  Error updateConfiguration(const ADCConfig &config);

  /**
   * @brief Get current ADC configuration
   *
   * Returns the current ADC configuration settings.
   *
   * @return Current ADCConfig structure
   *
   * @see ADCConfig
   */
  ADCConfig getConfiguration() const;

  /** @} */

  /**
   * @name ADC Reading Functions
   * @brief Functions for reading ADC values in various formats
   * @{
   */

  /**
   * @brief Read ADC value from specified channel with full reading information
   *
   * Performs a complete ADC reading from the specified channel, returning both
   * raw ADC value and converted voltage along with metadata.
   *
   * @param channel ADC channel to read from
   * @param reading Reference to ADCReading structure to store result
   * @return Error::SUCCESS if reading succeeds
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::INVALID_PARAMETER if channel is invalid
   * @return Error::I2C_ERROR if communication fails
   * @return Error::TIMEOUT if conversion times out
   *
   * @see ADCChannel
   * @see ADCReading
   */
  Error readChannel(ADCChannel channel, ADCReading &reading);

  /**
   * @brief Read raw ADC value from specified channel
   *
   * Performs an ADC reading and returns only the raw 16-bit signed value
   * without voltage conversion or metadata.
   *
   * @param channel ADC channel to read from
   * @param raw_value Reference to store raw ADC value
   * @return Error::SUCCESS if reading succeeds
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::INVALID_PARAMETER if channel is invalid
   * @return Error::I2C_ERROR if communication fails
   * @return Error::TIMEOUT if conversion times out
   */
  Error readChannelRaw(ADCChannel channel, int16_t &raw_value);

  /**
   * @brief Read voltage value from specified channel
   *
   * Performs an ADC reading and returns the converted voltage value
   * based on current gain settings.
   *
   * @param channel ADC channel to read from
   * @param voltage Reference to store voltage value
   * @return Error::SUCCESS if reading succeeds
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::INVALID_PARAMETER if channel is invalid
   * @return Error::I2C_ERROR if communication fails
   * @return Error::TIMEOUT if conversion times out
   */
  Error readChannelVoltage(ADCChannel channel, float &voltage);

  /**
   * @brief Read differential ADC value between specified channel pair
   *
   * Performs a differential measurement between two ADC inputs.
   * Only differential channels (DIFFERENTIAL_0_1, DIFFERENTIAL_0_3, etc.) are
   * valid.
   *
   * @param diff_channel Differential channel pair to read
   * @param reading Reference to ADCReading structure to store result
   * @return Error::SUCCESS if reading succeeds
   * @return Error::INVALID_PARAMETER if channel is not a differential channel
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::I2C_ERROR if communication fails
   * @return Error::TIMEOUT if conversion times out
   *
   * @see ADCChannel
   */
  Error readDifferential(ADCChannel diff_channel, ADCReading &reading);

  /** @} */

  /**
   * @name Asynchronous Conversion Functions
   * @brief Functions for non-blocking ADC conversions
   * @{
   */

  /**
   * @brief Start an asynchronous ADC conversion
   *
   * Initiates a conversion on the specified channel without blocking.
   * Use isConversionReady() to check completion and getConversionResult()
   * to retrieve the result.
   *
   * @param channel ADC channel to convert
   * @return Error::SUCCESS if conversion started successfully
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::INVALID_PARAMETER if channel is invalid
   * @return Error::I2C_ERROR if communication fails
   *
   * @see isConversionReady()
   * @see getConversionResult()
   */
  Error startConversion(ADCChannel channel);

  /**
   * @brief Check if asynchronous conversion is complete
   *
   * Checks the device status to determine if the previously started
   * conversion has completed.
   *
   * @return true if conversion is complete and result is ready
   * @return false if conversion is still in progress or no conversion started
   *
   * @note Returns false if device not initialized or no conversion in progress
   */
  bool isConversionReady();

  /**
   * @brief Get result from completed asynchronous conversion
   *
   * Retrieves the result from a previously started asynchronous conversion.
   * Call only after isConversionReady() returns true.
   *
   * @param reading Reference to ADCReading structure to store result
   * @return Error::SUCCESS if result retrieved successfully
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::CONFIGURATION_ERROR if no conversion in progress
   * @return Error::TIMEOUT if conversion not ready
   * @return Error::I2C_ERROR if communication fails
   */
  Error getConversionResult(ADCReading &reading);

  /** @} */

  /**
   * @name Multi-Channel Reading Functions
   * @brief Functions for reading multiple ADC channels efficiently
   * @{
   */

  /**
   * @brief Read all four single-ended channels (A0-A3)
   *
   * Sequentially reads all four single-ended ADC channels and stores
   * the results in the provided array.
   *
   * @param readings Array of 4 ADCReading structures to store results
   * @return Error::SUCCESS if all readings succeed
   * @return First error encountered if any reading fails
   *
   * @note Array must have space for exactly 4 ADCReading structures
   */
  Error readAllChannels(ADCReading readings[4]);

  /**
   * @brief Read multiple specified channels
   *
   * Sequentially reads the specified channels and stores results in the
   * provided array. Channels can be any combination of single-ended or
   * differential channels.
   *
   * @param channels Array of ADC channels to read
   * @param count Number of channels to read
   * @param readings Array to store reading results
   * @return Error::SUCCESS if all readings succeed
   * @return Error::INVALID_PARAMETER if pointers are null or count is 0
   * @return First error encountered if any reading fails
   *
   * @note Both arrays must have at least 'count' elements
   */
  Error readChannels(const ADCChannel *channels, size_t count,
                     ADCReading *readings);

  /** @} */

  /**
   * @name Configuration Functions
   * @brief Functions for configuring ADC parameters
   * @{
   */

  /**
   * @brief Update the programmable gain amplifier setting
   *
   * Changes the PGA setting which determines the input voltage range
   * and resolution of ADC readings.
   *
   * @param gain New gain amplifier setting
   * @return Error::SUCCESS if gain updated successfully
   * @return Error::INVALID_PARAMETER if gain value is invalid
   * @return Error::I2C_ERROR if communication fails
   *
   * @see GainAmplifier
   */
  Error updateGain(GainAmplifier gain);

  /**
   * @brief Get current gain amplifier setting
   *
   * @return Current GainAmplifier setting
   */
  GainAmplifier getGain() const;

  /**
   * @brief Update the ADC data rate (samples per second)
   *
   * Changes the conversion rate which affects both conversion time
   * and noise performance.
   *
   * @param rate New data rate setting
   * @return Error::SUCCESS if data rate updated successfully
   * @return Error::INVALID_PARAMETER if rate value is invalid
   * @return Error::I2C_ERROR if communication fails
   *
   * @see DataRate
   */
  Error updateDataRate(DataRate rate);

  /**
   * @brief Get current data rate setting
   *
   * @return Current DataRate setting
   */
  DataRate getDataRate() const;

  /**
   * @brief Update the operating mode (single-shot or continuous)
   *
   * Changes between single-shot conversions (default) and continuous
   * conversion mode.
   *
   * @param mode New operating mode
   * @return Error::SUCCESS if mode updated successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @see OperatingMode
   */
  Error updateOperatingMode(OperatingMode mode);

  /**
   * @brief Get current operating mode
   *
   * @return Current OperatingMode setting
   */
  OperatingMode getOperatingMode() const;

  /** @} */

  /**
   * @name Comparator and Alert Functions
   * @brief Functions for configuring and using the built-in comparator
   * @{
   */

  /**
   * @brief Update comparator mode and polarity
   *
   * Configures the comparator for traditional or window mode operation
   * and sets the alert pin polarity.
   *
   * @param mode Comparator mode (traditional or window)
   * @param polarity Alert pin polarity (active high or low)
   * @return Error::SUCCESS if configuration updated successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ComparatorMode
   * @see ComparatorPolarity
   */
  Error updateComparator(
      ComparatorMode mode,
      ComparatorPolarity polarity = ComparatorPolarity::ACTIVE_LOW);

  /**
   * @brief Set comparator threshold voltages
   *
   * Sets the high and low threshold voltages for the comparator.
   * In traditional mode, only high threshold is used. In window mode,
   * both thresholds define the window boundaries.
   *
   * @param low_threshold Low threshold voltage
   * @param high_threshold High threshold voltage
   * @return Error::SUCCESS if thresholds set successfully
   * @return Error::INVALID_PARAMETER if low >= high threshold
   * @return Error::I2C_ERROR if communication fails
   *
   * @note Threshold values are automatically converted to raw ADC values
   *       based on current gain setting
   */
  Error updateThresholds(float low_threshold, float high_threshold);

  /**
   * @brief Set comparator thresholds using configuration structure
   *
   * Sets threshold values using a ThresholdConfig structure.
   *
   * @param config Threshold configuration structure
   * @return Error::SUCCESS if thresholds set successfully
   * @return Error::INVALID_PARAMETER if configuration is invalid
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ThresholdConfig
   */
  Error updateThresholds(const ThresholdConfig &config);

  /**
   * @brief Enable the comparator with specified queue setting
   *
   * Enables the comparator and sets how many consecutive threshold
   * crossings are required before asserting the alert.
   *
   * @param queue Number of conversions required to assert alert
   * @return Error::SUCCESS if comparator enabled successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ComparatorQueue
   */
  Error
  enableComparator(ComparatorQueue queue = ComparatorQueue::ASSERT_AFTER_ONE);

  /**
   * @brief Disable the comparator
   *
   * Disables comparator functionality and alert generation.
   *
   * @return Error::SUCCESS if comparator disabled successfully
   * @return Error::I2C_ERROR if communication fails
   */
  Error disableComparator();

  /**
   * @brief Check if alert is currently active
   *
   * Reads the alert pin state to determine if an alert condition
   * is currently active.
   *
   * @return true if alert is active
   * @return false if alert is not active or alert pin not configured
   *
   * @note Returns false if alert pin is not configured (pin = 255)
   */
  bool isAlertActive();

  /**
   * @brief Clear active alert condition
   *
   * Clears the alert by reading the conversion register, which
   * acknowledges the alert condition.
   *
   * @return Error::SUCCESS if alert cleared successfully
   * @return Error::I2C_ERROR if communication fails
   */
  Error clearAlert();

  /**
   * @brief Update comparator latch setting
   *
   * Configures whether the alert latches (requires clearAlert() to reset)
   * or automatically clears when the condition is no longer met.
   *
   * @param latch Latching behavior setting
   * @return Error::SUCCESS if latch setting updated successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ComparatorLatch
   */
  Error updateComparatorLatch(ComparatorLatch latch);

  /**
   * @brief Update comparator queue setting
   *
   * Sets how many consecutive threshold crossings are required
   * before asserting the alert.
   *
   * @param queue Queue setting for alert assertion
   * @return Error::SUCCESS if queue setting updated successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ComparatorQueue
   */
  Error updateComparatorQueue(ComparatorQueue queue);

  /**
   * @brief Update comparator polarity setting
   *
   * Sets whether the alert pin is active high or active low.
   *
   * @param polarity Alert pin polarity
   * @return Error::SUCCESS if polarity updated successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @see ComparatorPolarity
   */
  Error updateComparatorPolarity(ComparatorPolarity polarity);

  /** @} */

  /**
   * @name Continuous Mode Functions
   * @brief Functions for continuous conversion mode operation
   * @{
   */

  /**
   * @brief Start continuous conversion mode on specified channel
   *
   * Switches the device to continuous conversion mode and begins
   * continuously converting the specified channel at the configured
   * data rate.
   *
   * @param channel ADC channel for continuous conversion
   * @return Error::SUCCESS if continuous mode started successfully
   * @return Error::NOT_INITIALIZED if device not initialized
   * @return Error::INVALID_PARAMETER if channel is invalid
   * @return Error::I2C_ERROR if communication fails
   *
   * @note Device remains in continuous mode until stopContinuousMode() is
   * called
   */
  Error startContinuousMode(ADCChannel channel);

  /**
   * @brief Stop continuous conversion mode
   *
   * Switches the device back to single-shot conversion mode.
   *
   * @return Error::SUCCESS if continuous mode stopped successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @note If not in continuous mode, this function succeeds without action
   */
  Error stopContinuousMode();

  /**
   * @brief Read the latest conversion result in continuous mode
   *
   * Reads the most recent conversion result from the device while
   * in continuous conversion mode.
   *
   * @param reading Reference to ADCReading structure to store result
   * @return Error::SUCCESS if reading retrieved successfully
   * @return Error::CONFIGURATION_ERROR if not in continuous mode
   * @return Error::I2C_ERROR if communication fails
   *
   * @note Device must be in continuous mode before calling this function
   */
  Error readContinuous(ADCReading &reading);

  /** @} */

  /**
   * @name Callback Functions
   * @brief Functions for setting up event callbacks
   * @{
   */

  /**
   * @brief Set callback function for conversion completion
   *
   * Sets a callback function that will be called when a conversion
   * completes (if callback system is implemented).
   *
   * @param callback Function pointer to conversion callback
   *
   * @see ConversionCallback
   */
  void updateConversionCallback(ConversionCallback callback);

  /**
   * @brief Set callback function for alert events
   *
   * Sets a callback function that will be called when an alert
   * condition occurs (if callback system is implemented).
   *
   * @param callback Function pointer to alert callback
   *
   * @see AlertCallback
   */
  void updateAlertCallback(AlertCallback callback);

  /**
   * @brief Set callback function for error events
   *
   * Sets a callback function that will be called when an error
   * occurs (if callback system is implemented).
   *
   * @param callback Function pointer to error callback
   *
   * @see ErrorCallback
   */
  void updateErrorCallback(ErrorCallback callback);

  /** @} */

  /**
   * @name Calibration and Utility Functions
   * @brief Functions for device calibration and information retrieval
   * @{
   */

  /**
   * @brief Perform device calibration
   *
   * Performs basic calibration procedures. This is a placeholder for
   * more sophisticated calibration routines that could be implemented.
   *
   * @return Error::SUCCESS if calibration completes
   *
   * @note Current implementation is a placeholder
   */
  Error calibrate();

  /**
   * @brief Perform device self-test
   *
   * Performs connectivity and register read/write tests to verify
   * proper device operation.
   *
   * @return Error::SUCCESS if all tests pass
   * @return Error::DEVICE_NOT_FOUND if device not responding
   * @return Error::I2C_ERROR if communication fails
   * @return Error::CONFIGURATION_ERROR if register test fails
   */
  Error performSelfTest();

  /**
   * @brief Get current voltage range based on gain setting
   *
   * Returns the full-scale voltage range for the current gain setting.
   *
   * @return Voltage range in volts (e.g., 6.144V for GAIN_TWOTHIRDS)
   */
  float getVoltageRange() const;

  /**
   * @brief Get voltage resolution per LSB for current gain setting
   *
   * Returns the voltage represented by one LSB (least significant bit)
   * of the ADC reading.
   *
   * @return Voltage per bit in volts
   */
  float getVoltageResolution() const;

  /**
   * @brief Get conversion time for current data rate setting
   *
   * Returns the expected conversion time in milliseconds based on
   * the current data rate setting.
   *
   * @return Conversion time in milliseconds
   */
  uint32_t getConversionTime() const;

  /** @} */

  /**
   * @name Low-Level Register Access
   * @brief Functions for direct register access (advanced users)
   * @{
   */

  /**
   * @brief Read value from device register
   *
   * Reads a 16-bit value from the specified device register.
   *
   * @param reg Register address to read from
   * @param value Reference to store register value
   * @return Error::SUCCESS if register read successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @warning Direct register access bypasses safety checks
   */
  Error readRegister(uint8_t reg, uint16_t &value);

  /**
   * @brief Write value to device register
   *
   * Writes a 16-bit value to the specified device register.
   *
   * @param reg Register address to write to
   * @param value Value to write to register
   * @return Error::SUCCESS if register written successfully
   * @return Error::I2C_ERROR if communication fails
   *
   * @warning Direct register access bypasses safety checks
   */
  Error writeRegister(uint8_t reg, uint16_t value);

  /**
   * @brief Read configuration register
   *
   * Convenience function to read the configuration register.
   *
   * @param config Reference to store configuration value
   * @return Error::SUCCESS if configuration read successfully
   * @return Error::I2C_ERROR if communication fails
   */
  Error readConfig(uint16_t &config);

  /**
   * @brief Write configuration register
   *
   * Convenience function to write the configuration register.
   *
   * @param config Configuration value to write
   * @return Error::SUCCESS if configuration written successfully
   * @return Error::I2C_ERROR if communication fails
   */
  Error writeConfig(uint16_t config);

  /** @} */

  /**
   * @name Utility Functions
   * @brief Utility functions for debugging and information display
   * @{
   */

  /**
   * @brief Get human-readable string for error code
   *
   * Converts an error code to a descriptive string for debugging
   * and user feedback.
   *
   * @param error Error code to convert
   * @return Pointer to static string describing the error
   */
  const char *getErrorString(Error error) const;

  /**
   * @brief Print current device status to stdout
   *
   * Prints comprehensive device status information including
   * initialization state, I2C address, mode, and statistics.
   */
  void printStatus() const;

  /**
   * @brief Print current device configuration to stdout
   *
   * Prints current ADC configuration including gain, data rate,
   * operating mode, and voltage range information.
   */
  void printConfiguration() const;

  /**
   * @brief Print ADC reading information to stdout
   *
   * Prints detailed information about an ADC reading including
   * channel, raw value, voltage, and validity.
   *
   * @param reading ADC reading to print
   */
  void printReading(const ADCReading &reading) const;

  /** @} */

  /**
   * @name Voltage Conversion Helpers
   * @brief Functions for converting between raw ADC values and voltages
   * @{
   */

  /**
   * @brief Convert raw ADC value to voltage
   *
   * Converts a raw 16-bit ADC value to voltage based on the current
   * gain amplifier setting.
   *
   * @param raw_value Raw ADC value to convert
   * @return Voltage value in volts
   */
  float rawToVoltage(int16_t raw_value) const;

  /**
   * @brief Convert voltage to raw ADC value
   *
   * Converts a voltage value to the corresponding raw ADC value
   * based on the current gain amplifier setting.
   *
   * @param voltage Voltage to convert
   * @return Raw ADC value (clamped to valid range)
   */
  int16_t voltageToRaw(float voltage) const;

  /**
   * @brief Check if raw ADC value represents a valid reading
   *
   * Checks if the raw value is within valid bounds and not at
   * the extreme values that might indicate overflow or error.
   *
   * @param raw_value Raw ADC value to validate
   * @return true if reading appears valid
   * @return false if reading may be invalid or saturated
   */
  bool isValidReading(int16_t raw_value) const;

  /** @} */

private:
  /**
   * @name Private Member Variables
   * @brief Internal state and configuration variables
   * @{
   */

  /** @brief I2C interface pointer */
  i2c_inst_t *_i2c;
  /** @brief I2C device address */
  uint8_t _address;
  /** @brief GPIO pin for alert functionality (255 = disabled) */
  uint _alert_pin;

  /** @brief Device initialization status */
  bool _initialized;
  /** @brief Current ADC configuration */
  ADCConfig _config;
  /** @brief Currently selected ADC channel */
  ADCChannel _current_channel;
  /** @brief Continuous conversion mode status */
  bool _continuous_mode;
  /** @brief Asynchronous conversion in progress flag */
  bool _conversion_in_progress;

  /** @brief Conversion completion callback function pointer */
  ConversionCallback _conversion_callback;
  /** @brief Alert event callback function pointer */
  AlertCallback _alert_callback;
  /** @brief Error event callback function pointer */
  ErrorCallback _error_callback;

  /** @brief Timestamp of last conversion start */
  absolute_time_t _last_conversion_time;
  /** @brief Total number of conversions performed */
  uint32_t _conversion_count;

  /** @brief Device configuration when using DeviceConfig constructor */
  DeviceConfig _device_config;

  /** @} */

  /**
   * @name Private Helper Functions
   * @brief Internal implementation functions (not part of public API)
   * @{
   */

  /** @brief Verify device presence and communication */
  Error _verifyDevice();
  /** @brief Configure device with default settings */
  Error _configureDefaults();
  /** @brief Validate I2C interface configuration */
  Error _validateI2C();
  /** @brief Initialize alert pin if configured */
  Error _initializeAlert();

  /** @brief Initialize device from DeviceConfig structure */
  Error _initializeFromDeviceConfig(const DeviceConfig &config);
  /** @brief Validate DeviceConfig structure */
  Error _validateDeviceConfig(const DeviceConfig &config);
  /** @brief Setup I2C hardware from configuration */
  Error _setupI2CHardware(const DeviceConfig &config);

  /** @brief Low-level I2C register write */
  Error _i2cWrite(uint8_t reg, uint16_t value);
  /** @brief Low-level I2C register read */
  Error _i2cRead(uint8_t reg, uint16_t &value);
  /** @brief Low-level I2C multi-byte write */
  Error _i2cWriteBytes(uint8_t reg, const uint8_t *data, size_t length);
  /** @brief Low-level I2C multi-byte read */
  Error _i2cReadBytes(uint8_t reg, uint8_t *buffer, size_t length);

  /** @brief Build configuration register value for current channel */
  uint16_t _buildConfigRegister() const;
  /** @brief Build configuration register value for specified channel */
  uint16_t _buildConfigRegister(ADCChannel channel) const;
  /** @brief Update device configuration register */
  Error _updateConfiguration();
  /** @brief Set input multiplexer for specified channel */
  Error _setChannelMux(ADCChannel channel);

  /** @brief Start single conversion on specified channel */
  Error _startSingleConversion(ADCChannel channel);
  /** @brief Wait for conversion to complete with timeout */
  Error _waitForConversion();
  /** @brief Read conversion result from device */
  Error _readConversionResult(int16_t &raw_value);
  /** @brief Create ADCReading structure from raw value */
  ADCReading _createReading(int16_t raw_value, ADCChannel channel);

  /** @brief Set raw threshold values in device registers */
  Error _setRawThresholds(int16_t low, int16_t high);
  /** @brief Get raw threshold values from device registers */
  Error _getRawThresholds(int16_t &low, int16_t &high);

  /** @brief Validate ADC channel parameter */
  bool _isValidChannel(ADCChannel channel) const;
  /** @brief Validate gain amplifier parameter */
  bool _isValidGain(GainAmplifier gain) const;
  /** @brief Validate data rate parameter */
  bool _isValidDataRate(DataRate rate) const;

  /** @brief Update internal channel state */
  void _updateChannelState(ADCChannel channel);
  /** @brief Call conversion completion callback if set */
  void _callConversionCallback(const ADCReading &reading);
  /** @brief Call alert callback if set */
  void _callAlertCallback(ADCChannel channel, bool alert_active);
  /** @brief Call error callback if set */
  void _callErrorCallback(Error error, const char *message = nullptr);

  /** @brief Handle alert condition */
  void _handleAlert();
  /** @brief Check alert pin state */
  bool _checkAlertPin();

  /** @brief Print debug message */
  void _debugPrint(const char *message) const;
  /** @brief Print debug register information */
  void _debugPrintRegister(uint8_t reg, uint16_t value) const;
  /** @brief Print debug configuration information */
  void _debugPrintConfig(uint16_t config) const;

  /** @} */
};

/**
 * @name Global Utility Functions
 * @brief Standalone utility functions for ADS1115 operations
 * @{
 */

/**
 * @brief Convert error code to descriptive string
 *
 * @param error Error code to convert
 * @return Pointer to static string describing the error
 */
const char *errorToString(Error error);

/**
 * @brief Convert ADC channel to descriptive string
 *
 * @param channel ADC channel to convert
 * @return Pointer to static string describing the channel
 */
const char *channelToString(ADCChannel channel);

/**
 * @brief Convert gain amplifier setting to descriptive string
 *
 * @param gain Gain setting to convert
 * @return Pointer to static string describing the gain
 */
const char *gainToString(GainAmplifier gain);

/**
 * @brief Convert data rate setting to descriptive string
 *
 * @param rate Data rate to convert
 * @return Pointer to static string describing the data rate
 */
const char *dataRateToString(DataRate rate);

/**
 * @brief Convert operating mode to descriptive string
 *
 * @param mode Operating mode to convert
 * @return Pointer to static string describing the mode
 */
const char *operatingModeToString(OperatingMode mode);

/** @} */

/**
 * @name Conversion Helper Functions
 * @brief Functions for converting between different representations
 * @{
 */

/**
 * @brief Convert array index to ADC channel
 *
 * @param index Array index (0-3)
 * @return Corresponding ADC channel (A0-A3)
 */
ADCChannel indexToChannel(uint8_t index);

/**
 * @brief Convert ADC channel to array index
 *
 * @param channel ADC channel to convert
 * @return Array index representation
 */
uint8_t channelToIndex(ADCChannel channel);

/**
 * @brief Determine optimal gain setting for voltage range
 *
 * @param max_voltage Maximum expected voltage
 * @return Optimal gain amplifier setting
 */
GainAmplifier voltageRangeToGain(float max_voltage);

/**
 * @brief Determine data rate setting from frequency
 *
 * @param frequency Desired sampling frequency in Hz
 * @return Closest available data rate setting
 */
DataRate frequencyToDataRate(uint16_t frequency);

/** @} */

/**
 * @name Voltage Range Helper Functions
 * @brief Functions for working with voltage ranges and gain settings
 * @{
 */

/**
 * @brief Get maximum voltage for gain setting
 *
 * @param gain Gain amplifier setting
 * @return Maximum positive voltage
 */
float getMaxVoltage(GainAmplifier gain);

/**
 * @brief Get minimum voltage for gain setting
 *
 * @param gain Gain amplifier setting
 * @return Maximum negative voltage
 */
float getMinVoltage(GainAmplifier gain);

/**
 * @brief Check if voltage is within range for gain setting
 *
 * @param voltage Voltage to check
 * @param gain Gain amplifier setting
 * @return true if voltage is within range
 */
bool isVoltageInRange(float voltage, GainAmplifier gain);

/** @} */

} // namespace ADS1115
