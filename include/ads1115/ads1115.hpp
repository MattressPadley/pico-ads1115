#pragma once

#include "ads1115_registers.hpp"
#include "ads1115_types.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

namespace ADS1115 {

class ADS1115Device {
public:
    // Constructor
    explicit ADS1115Device(i2c_inst_t* i2c_instance, 
                          uint8_t device_address = DEFAULT_I2C_ADDRESS,
                          uint alert_pin = 255); // Invalid pin = no alert

    // Destructor
    ~ADS1115Device();

    // Device initialization and management
    Error begin();
    Error reset();
    bool isConnected();
    DeviceStatus getStatus();
    Error setConfiguration(const ADCConfig& config);
    ADCConfig getConfiguration() const;

    // ADC reading functions
    Error readChannel(ADCChannel channel, ADCReading& reading);
    Error readChannelRaw(ADCChannel channel, int16_t& raw_value);
    Error readChannelVoltage(ADCChannel channel, float& voltage);
    Error readDifferential(ADCChannel diff_channel, ADCReading& reading);
    
    // Asynchronous conversion functions
    Error startConversion(ADCChannel channel);
    bool isConversionReady();
    Error getConversionResult(ADCReading& reading);
    
    // Multi-channel reading
    Error readAllChannels(ADCReading readings[4]);
    Error readChannels(const ADCChannel* channels, size_t count, ADCReading* readings);
    
    // Configuration functions
    Error setGain(GainAmplifier gain);
    GainAmplifier getGain() const;
    Error setDataRate(DataRate rate);
    DataRate getDataRate() const;
    Error setOperatingMode(OperatingMode mode);
    OperatingMode getOperatingMode() const;
    
    // Comparator and alert functions
    Error setComparator(ComparatorMode mode, ComparatorPolarity polarity = ComparatorPolarity::ACTIVE_LOW);
    Error setThresholds(float low_threshold, float high_threshold);
    Error setThresholds(const ThresholdConfig& config);
    Error enableComparator(ComparatorQueue queue = ComparatorQueue::ASSERT_AFTER_ONE);
    Error disableComparator();
    bool isAlertActive();
    Error clearAlert();
    
    // Advanced comparator configuration
    Error setComparatorLatch(ComparatorLatch latch);
    Error setComparatorQueue(ComparatorQueue queue);
    Error setComparatorPolarity(ComparatorPolarity polarity);
    
    // Continuous mode functions
    Error startContinuousMode(ADCChannel channel);
    Error stopContinuousMode();
    Error readContinuous(ADCReading& reading);
    
    // Callback functions
    void setConversionCallback(ConversionCallback callback);
    void setAlertCallback(AlertCallback callback);
    void setErrorCallback(ErrorCallback callback);
    
    // Calibration and utility functions
    Error calibrate();
    Error performSelfTest();
    float getVoltageRange() const;
    float getVoltageResolution() const;
    uint32_t getConversionTime() const;
    
    // Low-level register access
    Error readRegister(uint8_t reg, uint16_t& value);
    Error writeRegister(uint8_t reg, uint16_t value);
    Error readConfig(uint16_t& config);
    Error writeConfig(uint16_t config);
    
    // Utility functions
    const char* getErrorString(Error error) const;
    void printStatus() const;
    void printConfiguration() const;
    void printReading(const ADCReading& reading) const;
    
    // Voltage conversion helpers
    float rawToVoltage(int16_t raw_value) const;
    int16_t voltageToRaw(float voltage) const;
    bool isValidReading(int16_t raw_value) const;
    
private:
    // Hardware interface
    i2c_inst_t* _i2c;
    uint8_t _address;
    uint _alert_pin;
    
    // State tracking
    bool _initialized;
    ADCConfig _config;
    ADCChannel _current_channel;
    bool _continuous_mode;
    bool _conversion_in_progress;
    
    // Callbacks
    ConversionCallback _conversion_callback;
    AlertCallback _alert_callback;
    ErrorCallback _error_callback;
    
    // Timing and performance
    absolute_time_t _last_conversion_time;
    uint32_t _conversion_count;
    
    // Internal helper functions
    Error _verifyDevice();
    Error _configureDefaults();
    Error _validateI2C();
    Error _initializeAlert();
    
    // I2C communication helpers
    Error _i2cWrite(uint8_t reg, uint16_t value);
    Error _i2cRead(uint8_t reg, uint16_t& value);
    Error _i2cWriteBytes(uint8_t reg, const uint8_t* data, size_t length);
    Error _i2cReadBytes(uint8_t reg, uint8_t* buffer, size_t length);
    
    // Configuration helpers
    uint16_t _buildConfigRegister() const;
    uint16_t _buildConfigRegister(ADCChannel channel) const;
    Error _updateConfiguration();
    Error _setChannelMux(ADCChannel channel);
    
    // Conversion helpers
    Error _startSingleConversion(ADCChannel channel);
    Error _waitForConversion();
    Error _readConversionResult(int16_t& raw_value);
    ADCReading _createReading(int16_t raw_value, ADCChannel channel);
    
    // Threshold helpers
    Error _setRawThresholds(int16_t low, int16_t high);
    Error _getRawThresholds(int16_t& low, int16_t& high);
    
    // Validation helpers
    bool _isValidChannel(ADCChannel channel) const;
    bool _isValidGain(GainAmplifier gain) const;
    bool _isValidDataRate(DataRate rate) const;
    
    // State management
    void _updateChannelState(ADCChannel channel);
    void _callConversionCallback(const ADCReading& reading);
    void _callAlertCallback(ADCChannel channel, bool alert_active);
    void _callErrorCallback(Error error, const char* message = nullptr);
    
    // Alert and interrupt handling
    void _handleAlert();
    bool _checkAlertPin();
    
    // Debug helpers
    void _debugPrint(const char* message) const;
    void _debugPrintRegister(uint8_t reg, uint16_t value) const;
    void _debugPrintConfig(uint16_t config) const;
};

// Global utility functions
const char* errorToString(Error error);
const char* channelToString(ADCChannel channel);
const char* gainToString(GainAmplifier gain);
const char* dataRateToString(DataRate rate);
const char* operatingModeToString(OperatingMode mode);

// Conversion helpers
ADCChannel indexToChannel(uint8_t index);
uint8_t channelToIndex(ADCChannel channel);
GainAmplifier voltageRangeToGain(float max_voltage);
DataRate frequencyToDataRate(uint16_t frequency);

// Voltage range helpers
float getMaxVoltage(GainAmplifier gain);
float getMinVoltage(GainAmplifier gain);
bool isVoltageInRange(float voltage, GainAmplifier gain);

} // namespace ADS1115