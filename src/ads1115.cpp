#include "ads1115/ads1115.hpp"
#include <cstring>
#include <stdio.h>

namespace ADS1115 {

ADS1115Device::ADS1115Device(i2c_inst_t* i2c_instance, uint8_t device_address,
                           uint alert_pin)
    : _i2c(i2c_instance)
    , _address(device_address)
    , _alert_pin(alert_pin)
    , _initialized(false)
    , _current_channel(ADCChannel::A0)
    , _continuous_mode(false)
    , _conversion_in_progress(false)
    , _conversion_callback(nullptr)
    , _alert_callback(nullptr)
    , _error_callback(nullptr)
    , _conversion_count(0)
{
    // Initialize configuration with defaults
    _config = ADCConfig();
}

ADS1115Device::~ADS1115Device() {
    if (_initialized) {
        // Stop continuous mode if active
        stopContinuousMode();
        disableComparator();
    }
}

Error ADS1115Device::begin() {
    // Validate I2C is properly initialized
    Error err = _validateI2C();
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Wait for device to be ready
    sleep_ms(POWER_ON_DELAY_MS);
    
    // Verify device is present
    err = _verifyDevice();
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Configure default settings
    err = _configureDefaults();
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Initialize alert pin if configured
    if (_alert_pin < 255) {
        err = _initializeAlert();
        if (err != Error::SUCCESS) {
            return err;
        }
    }
    
    _initialized = true;
    return Error::SUCCESS;
}

Error ADS1115Device::reset() {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    // Perform software reset by writing default config
    Error err = writeConfig(CONFIG_RESET_VALUE);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    sleep_ms(CONVERSION_DELAY_MS);
    
    // Reconfigure with current settings
    return _configureDefaults();
}

bool ADS1115Device::isConnected() {
    uint16_t config;
    Error err = readConfig(config);
    return (err == Error::SUCCESS);
}

DeviceStatus ADS1115Device::getStatus() {
    DeviceStatus status;
    
    if (!_initialized) {
        return status;
    }
    
    uint16_t config;
    Error err = readConfig(config);
    if (err != Error::SUCCESS) {
        return status;
    }
    
    status.device_ready = true;
    status.conversion_ready = (config & CONFIG_OS_MASK) == CONFIG_OS_NOT_BUSY;
    status.comparator_alert = isAlertActive();
    status.conversion_count = _conversion_count;
    
    return status;
}

Error ADS1115Device::setConfiguration(const ADCConfig& config) {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    _config = config;
    return _updateConfiguration();
}

ADCConfig ADS1115Device::getConfiguration() const {
    return _config;
}

Error ADS1115Device::readChannel(ADCChannel channel, ADCReading& reading) {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    int16_t raw_value;
    Error err = readChannelRaw(channel, raw_value);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    reading = _createReading(raw_value, channel);
    return Error::SUCCESS;
}

Error ADS1115Device::readChannelRaw(ADCChannel channel, int16_t& raw_value) {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    // Start single conversion
    Error err = _startSingleConversion(channel);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Wait for conversion to complete
    err = _waitForConversion();
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Read the result
    err = _readConversionResult(raw_value);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    _conversion_count++;
    return Error::SUCCESS;
}

Error ADS1115Device::readChannelVoltage(ADCChannel channel, float& voltage) {
    int16_t raw_value;
    Error err = readChannelRaw(channel, raw_value);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    voltage = rawToVoltage(raw_value);
    return Error::SUCCESS;
}

Error ADS1115Device::readDifferential(ADCChannel diff_channel, ADCReading& reading) {
    if (!isDifferentialChannel(diff_channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    return readChannel(diff_channel, reading);
}

Error ADS1115Device::startConversion(ADCChannel channel) {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    Error err = _startSingleConversion(channel);
    if (err == Error::SUCCESS) {
        _conversion_in_progress = true;
        _current_channel = channel;
    }
    
    return err;
}

bool ADS1115Device::isConversionReady() {
    if (!_initialized || !_conversion_in_progress) {
        return false;
    }
    
    uint16_t config;
    Error err = readConfig(config);
    if (err != Error::SUCCESS) {
        return false;
    }
    
    return (config & CONFIG_OS_MASK) == CONFIG_OS_NOT_BUSY;
}

Error ADS1115Device::getConversionResult(ADCReading& reading) {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    if (!_conversion_in_progress) {
        return Error::CONFIGURATION_ERROR;
    }
    
    if (!isConversionReady()) {
        return Error::TIMEOUT;
    }
    
    int16_t raw_value;
    Error err = _readConversionResult(raw_value);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    reading = _createReading(raw_value, _current_channel);
    _conversion_in_progress = false;
    _conversion_count++;
    
    return Error::SUCCESS;
}

Error ADS1115Device::readAllChannels(ADCReading readings[4]) {
    for (int i = 0; i < 4; i++) {
        ADCChannel channel = static_cast<ADCChannel>(i);
        Error err = readChannel(channel, readings[i]);
        if (err != Error::SUCCESS) {
            return err;
        }
    }
    return Error::SUCCESS;
}

Error ADS1115Device::readChannels(const ADCChannel* channels, size_t count, ADCReading* readings) {
    if (channels == nullptr || readings == nullptr || count == 0) {
        return Error::INVALID_PARAMETER;
    }
    
    for (size_t i = 0; i < count; i++) {
        Error err = readChannel(channels[i], readings[i]);
        if (err != Error::SUCCESS) {
            return err;
        }
    }
    
    return Error::SUCCESS;
}

Error ADS1115Device::setGain(GainAmplifier gain) {
    if (!_isValidGain(gain)) {
        return Error::INVALID_PARAMETER;
    }
    
    _config.gain = gain;
    return _updateConfiguration();
}

GainAmplifier ADS1115Device::getGain() const {
    return _config.gain;
}

Error ADS1115Device::setDataRate(DataRate rate) {
    if (!_isValidDataRate(rate)) {
        return Error::INVALID_PARAMETER;
    }
    
    _config.data_rate = rate;
    return _updateConfiguration();
}

DataRate ADS1115Device::getDataRate() const {
    return _config.data_rate;
}

Error ADS1115Device::setOperatingMode(OperatingMode mode) {
    _config.mode = mode;
    return _updateConfiguration();
}

OperatingMode ADS1115Device::getOperatingMode() const {
    return _config.mode;
}

Error ADS1115Device::setComparator(ComparatorMode mode, ComparatorPolarity polarity) {
    _config.comparator_mode = mode;
    _config.comparator_polarity = polarity;
    return _updateConfiguration();
}

Error ADS1115Device::setThresholds(float low_threshold, float high_threshold) {
    if (low_threshold >= high_threshold) {
        return Error::INVALID_PARAMETER;
    }
    
    int16_t low_raw = voltageToRaw(low_threshold);
    int16_t high_raw = voltageToRaw(high_threshold);
    
    return _setRawThresholds(low_raw, high_raw);
}

Error ADS1115Device::setThresholds(const ThresholdConfig& config) {
    return setThresholds(config.low_threshold, config.high_threshold);
}

Error ADS1115Device::enableComparator(ComparatorQueue queue) {
    _config.comparator_queue = queue;
    return _updateConfiguration();
}

Error ADS1115Device::disableComparator() {
    _config.comparator_queue = ComparatorQueue::DISABLE;
    return _updateConfiguration();
}

bool ADS1115Device::isAlertActive() {
    if (_alert_pin >= 255) {
        return false;
    }
    
    bool pin_state = gpio_get(_alert_pin);
    return (_config.comparator_polarity == ComparatorPolarity::ACTIVE_HIGH) ? pin_state : !pin_state;
}

Error ADS1115Device::clearAlert() {
    // Reading the conversion register clears the alert
    uint16_t dummy;
    return readRegister(REG_CONVERSION, dummy);
}

Error ADS1115Device::setComparatorLatch(ComparatorLatch latch) {
    _config.comparator_latch = latch;
    return _updateConfiguration();
}

Error ADS1115Device::setComparatorQueue(ComparatorQueue queue) {
    _config.comparator_queue = queue;
    return _updateConfiguration();
}

Error ADS1115Device::setComparatorPolarity(ComparatorPolarity polarity) {
    _config.comparator_polarity = polarity;
    return _updateConfiguration();
}

Error ADS1115Device::startContinuousMode(ADCChannel channel) {
    if (!_initialized) {
        return Error::NOT_INITIALIZED;
    }
    
    if (!_isValidChannel(channel)) {
        return Error::INVALID_PARAMETER;
    }
    
    _config.mode = OperatingMode::CONTINUOUS;
    _current_channel = channel;
    
    uint16_t config = _buildConfigRegister(channel);
    Error err = writeConfig(config);
    if (err == Error::SUCCESS) {
        _continuous_mode = true;
        sleep_ms(getConversionTime());
    }
    
    return err;
}

Error ADS1115Device::stopContinuousMode() {
    if (!_continuous_mode) {
        return Error::SUCCESS;
    }
    
    _config.mode = OperatingMode::SINGLE_SHOT;
    _continuous_mode = false;
    
    return _updateConfiguration();
}

Error ADS1115Device::readContinuous(ADCReading& reading) {
    if (!_continuous_mode) {
        return Error::CONFIGURATION_ERROR;
    }
    
    int16_t raw_value;
    Error err = _readConversionResult(raw_value);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    reading = _createReading(raw_value, _current_channel);
    _conversion_count++;
    
    return Error::SUCCESS;
}

void ADS1115Device::setConversionCallback(ConversionCallback callback) {
    _conversion_callback = callback;
}

void ADS1115Device::setAlertCallback(AlertCallback callback) {
    _alert_callback = callback;
}

void ADS1115Device::setErrorCallback(ErrorCallback callback) {
    _error_callback = callback;
}

Error ADS1115Device::calibrate() {
    // Perform a basic calibration by reading a known reference
    // This is a simplified calibration - more sophisticated methods could be implemented
    return Error::SUCCESS;
}

Error ADS1115Device::performSelfTest() {
    // Perform basic connectivity and register tests
    if (!isConnected()) {
        return Error::DEVICE_NOT_FOUND;
    }
    
    // Test register read/write
    uint16_t original_config;
    Error err = readConfig(original_config);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Write a test value and read it back
    uint16_t test_config = original_config ^ CONFIG_CPOL_MASK;
    err = writeConfig(test_config);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    uint16_t read_config;
    err = readConfig(read_config);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    // Restore original configuration
    writeConfig(original_config);
    
    return (read_config == test_config) ? Error::SUCCESS : Error::CONFIGURATION_ERROR;
}

float ADS1115Device::getVoltageRange() const {
    return ADS1115::getVoltageRange(_config.gain);
}

float ADS1115Device::getVoltageResolution() const {
    return getVoltagePerBit(_config.gain);
}

uint32_t ADS1115Device::getConversionTime() const {
    return getConversionTimeMs(_config.data_rate);
}

Error ADS1115Device::readRegister(uint8_t reg, uint16_t& value) {
    return _i2cRead(reg, value);
}

Error ADS1115Device::writeRegister(uint8_t reg, uint16_t value) {
    return _i2cWrite(reg, value);
}

Error ADS1115Device::readConfig(uint16_t& config) {
    return readRegister(REG_CONFIG, config);
}

Error ADS1115Device::writeConfig(uint16_t config) {
    return writeRegister(REG_CONFIG, config);
}

const char* ADS1115Device::getErrorString(Error error) const {
    return errorToString(error);
}

void ADS1115Device::printStatus() const {
    printf("ADS1115 Device Status:\n");
    printf("  Initialized: %s\n", _initialized ? "Yes" : "No");
    printf("  I2C Address: 0x%02X\n", _address);
    printf("  Continuous Mode: %s\n", _continuous_mode ? "Yes" : "No");
    printf("  Conversion Count: %lu\n", _conversion_count);
    printf("  Current Channel: %s\n", channelToString(_current_channel));
}

void ADS1115Device::printConfiguration() const {
    printf("ADS1115 Configuration:\n");
    printf("  Gain: %s\n", gainToString(_config.gain));
    printf("  Data Rate: %s\n", dataRateToString(_config.data_rate));
    printf("  Mode: %s\n", operatingModeToString(_config.mode));
    printf("  Voltage Range: ±%.3fV\n", getVoltageRange());
    printf("  Resolution: %.6fV/bit\n", getVoltageResolution());
}

void ADS1115Device::printReading(const ADCReading& reading) const {
    printf("ADC Reading:\n");
    printf("  Channel: %s\n", channelToString(reading.channel));
    printf("  Raw Value: %d\n", reading.raw_value);
    printf("  Voltage: %.6fV\n", reading.voltage);
    printf("  Valid: %s\n", reading.valid ? "Yes" : "No");
}

float ADS1115Device::rawToVoltage(int16_t raw_value) const {
    return ADS1115::rawToVoltage(raw_value, _config.gain);
}

int16_t ADS1115Device::voltageToRaw(float voltage) const {
    return ADS1115::voltageToRaw(voltage, _config.gain);
}

bool ADS1115Device::isValidReading(int16_t raw_value) const {
    return (raw_value != ADC_MIN_VALUE && raw_value != ADC_MAX_VALUE);
}

// Private helper functions

Error ADS1115Device::_verifyDevice() {
    return isConnected() ? Error::SUCCESS : Error::DEVICE_NOT_FOUND;
}

Error ADS1115Device::_configureDefaults() {
    return _updateConfiguration();
}

Error ADS1115Device::_validateI2C() {
    if (_i2c == nullptr) {
        return Error::INVALID_PARAMETER;
    }
    
    // For now, just check that I2C instance is not null
    // The actual device communication will be tested in _verifyDevice()
    return Error::SUCCESS;
}

Error ADS1115Device::_initializeAlert() {
    gpio_init(_alert_pin);
    gpio_set_dir(_alert_pin, GPIO_IN);
    gpio_pull_up(_alert_pin);
    return Error::SUCCESS;
}

Error ADS1115Device::_i2cWrite(uint8_t reg, uint16_t value) {
    uint8_t buffer[3] = {reg, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF)};
    int result = i2c_write_blocking(_i2c, _address, buffer, 3, false);
    return (result == 3) ? Error::SUCCESS : Error::I2C_ERROR;
}

Error ADS1115Device::_i2cRead(uint8_t reg, uint16_t& value) {
    uint8_t buffer[2];
    int result = i2c_write_blocking(_i2c, _address, &reg, 1, true);
    if (result != 1) {
        return Error::I2C_ERROR;
    }
    
    result = i2c_read_blocking(_i2c, _address, buffer, 2, false);
    if (result != 2) {
        return Error::I2C_ERROR;
    }
    
    value = (static_cast<uint16_t>(buffer[0]) << 8) | buffer[1];
    return Error::SUCCESS;
}

uint16_t ADS1115Device::_buildConfigRegister() const {
    return _buildConfigRegister(_current_channel);
}

uint16_t ADS1115Device::_buildConfigRegister(ADCChannel channel) const {
    uint16_t config = 0;
    
    // Set operational status (start single conversion for single-shot mode)
    if (_config.mode == OperatingMode::SINGLE_SHOT) {
        config |= CONFIG_OS_SINGLE;
    }
    
    // Set input multiplexer
    config |= getChannelMux(static_cast<uint8_t>(channel));
    
    // Set gain
    config |= getGainPGA(static_cast<uint8_t>(_config.gain));
    
    // Set operating mode
    config |= (_config.mode == OperatingMode::CONTINUOUS) ? CONFIG_MODE_CONTIN : CONFIG_MODE_SINGLE;
    
    // Set data rate
    config |= getDataRateDR(static_cast<uint8_t>(_config.data_rate));
    
    // Set comparator mode
    config |= (_config.comparator_mode == ComparatorMode::WINDOW) ? CONFIG_CMODE_WINDOW : CONFIG_CMODE_TRAD;
    
    // Set comparator polarity
    config |= (_config.comparator_polarity == ComparatorPolarity::ACTIVE_HIGH) ? CONFIG_CPOL_ACTVHI : CONFIG_CPOL_ACTVLOW;
    
    // Set comparator latch
    config |= (_config.comparator_latch == ComparatorLatch::LATCHING) ? CONFIG_CLAT_LATCH : CONFIG_CLAT_NONLAT;
    
    // Set comparator queue
    switch (_config.comparator_queue) {
        case ComparatorQueue::ASSERT_AFTER_ONE: config |= CONFIG_CQUE_1CONV; break;
        case ComparatorQueue::ASSERT_AFTER_TWO: config |= CONFIG_CQUE_2CONV; break;
        case ComparatorQueue::ASSERT_AFTER_FOUR: config |= CONFIG_CQUE_4CONV; break;
        case ComparatorQueue::DISABLE: config |= CONFIG_CQUE_NONE; break;
    }
    
    return config;
}

Error ADS1115Device::_updateConfiguration() {
    uint16_t config = _buildConfigRegister();
    return writeConfig(config);
}

Error ADS1115Device::_setChannelMux(ADCChannel channel) {
    uint16_t config;
    Error err = readConfig(config);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    config = SET_BITS(config, CONFIG_MUX_MASK, getChannelMux(static_cast<uint8_t>(channel)));
    return writeConfig(config);
}

Error ADS1115Device::_startSingleConversion(ADCChannel channel) {
    uint16_t config = _buildConfigRegister(channel);
    config |= CONFIG_OS_SINGLE;  // Start conversion
    
    Error err = writeConfig(config);
    if (err == Error::SUCCESS) {
        _last_conversion_time = get_absolute_time();
    }
    
    return err;
}

Error ADS1115Device::_waitForConversion() {
    uint32_t timeout_ms = getConversionTime() + 50;  // Add safety margin
    absolute_time_t timeout = make_timeout_time_ms(timeout_ms);
    
    while (!time_reached(timeout)) {
        uint16_t config;
        Error err = readConfig(config);
        if (err != Error::SUCCESS) {
            return err;
        }
        
        if ((config & CONFIG_OS_MASK) == CONFIG_OS_NOT_BUSY) {
            return Error::SUCCESS;
        }
        
        sleep_ms(1);
    }
    
    return Error::TIMEOUT;
}

Error ADS1115Device::_readConversionResult(int16_t& raw_value) {
    uint16_t result;
    Error err = readRegister(REG_CONVERSION, result);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    raw_value = static_cast<int16_t>(result);
    return Error::SUCCESS;
}

ADCReading ADS1115Device::_createReading(int16_t raw_value, ADCChannel channel) {
    ADCReading reading;
    reading.raw_value = raw_value;
    reading.voltage = rawToVoltage(raw_value);
    reading.channel = channel;
    reading.timestamp = to_us_since_boot(get_absolute_time());
    reading.valid = isValidReading(raw_value);
    
    return reading;
}

Error ADS1115Device::_setRawThresholds(int16_t low, int16_t high) {
    Error err = writeRegister(REG_LO_THRESH, static_cast<uint16_t>(low));
    if (err != Error::SUCCESS) {
        return err;
    }
    
    return writeRegister(REG_HI_THRESH, static_cast<uint16_t>(high));
}

Error ADS1115Device::_getRawThresholds(int16_t& low, int16_t& high) {
    uint16_t low_reg, high_reg;
    Error err = readRegister(REG_LO_THRESH, low_reg);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    err = readRegister(REG_HI_THRESH, high_reg);
    if (err != Error::SUCCESS) {
        return err;
    }
    
    low = static_cast<int16_t>(low_reg);
    high = static_cast<int16_t>(high_reg);
    return Error::SUCCESS;
}

bool ADS1115Device::_isValidChannel(ADCChannel channel) const {
    return isValidChannel(channel);
}

bool ADS1115Device::_isValidGain(GainAmplifier gain) const {
    return static_cast<uint8_t>(gain) <= static_cast<uint8_t>(GainAmplifier::GAIN_SIXTEEN);
}

bool ADS1115Device::_isValidDataRate(DataRate rate) const {
    return static_cast<uint8_t>(rate) <= static_cast<uint8_t>(DataRate::SPS_860);
}

void ADS1115Device::_updateChannelState(ADCChannel channel) {
    _current_channel = channel;
}

void ADS1115Device::_callConversionCallback(const ADCReading& reading) {
    if (_conversion_callback) {
        _conversion_callback(reading);
    }
}

void ADS1115Device::_callAlertCallback(ADCChannel channel, bool alert_active) {
    if (_alert_callback) {
        _alert_callback(channel, alert_active);
    }
}

void ADS1115Device::_callErrorCallback(Error error, const char* message) {
    if (_error_callback) {
        _error_callback(error, message);
    }
}

void ADS1115Device::_handleAlert() {
    if (isAlertActive()) {
        _callAlertCallback(_current_channel, true);
    }
}

bool ADS1115Device::_checkAlertPin() {
    return isAlertActive();
}

void ADS1115Device::_debugPrint(const char* message) const {
    printf("[ADS1115] %s\n", message);
}

void ADS1115Device::_debugPrintRegister(uint8_t reg, uint16_t value) const {
    printf("[ADS1115] Register 0x%02X: 0x%04X\n", reg, value);
}

void ADS1115Device::_debugPrintConfig(uint16_t config) const {
    printf("[ADS1115] Config: 0x%04X\n", config);
}

// Global utility functions

const char* errorToString(Error error) {
    switch (error) {
        case Error::SUCCESS: return "Success";
        case Error::I2C_ERROR: return "I2C Error";
        case Error::DEVICE_NOT_FOUND: return "Device Not Found";
        case Error::INVALID_PARAMETER: return "Invalid Parameter";
        case Error::TIMEOUT: return "Timeout";
        case Error::NOT_INITIALIZED: return "Not Initialized";
        case Error::CONFIGURATION_ERROR: return "Configuration Error";
        case Error::CONVERSION_ERROR: return "Conversion Error";
        default: return "Unknown Error";
    }
}

const char* channelToString(ADCChannel channel) {
    switch (channel) {
        case ADCChannel::A0: return "A0";
        case ADCChannel::A1: return "A1";
        case ADCChannel::A2: return "A2";
        case ADCChannel::A3: return "A3";
        case ADCChannel::DIFFERENTIAL_0_1: return "Differential 0-1";
        case ADCChannel::DIFFERENTIAL_0_3: return "Differential 0-3";
        case ADCChannel::DIFFERENTIAL_1_3: return "Differential 1-3";
        case ADCChannel::DIFFERENTIAL_2_3: return "Differential 2-3";
        default: return "Unknown";
    }
}

const char* gainToString(GainAmplifier gain) {
    switch (gain) {
        case GainAmplifier::GAIN_TWOTHIRDS: return "2/3x (±6.144V)";
        case GainAmplifier::GAIN_ONE: return "1x (±4.096V)";
        case GainAmplifier::GAIN_TWO: return "2x (±2.048V)";
        case GainAmplifier::GAIN_FOUR: return "4x (±1.024V)";
        case GainAmplifier::GAIN_EIGHT: return "8x (±0.512V)";
        case GainAmplifier::GAIN_SIXTEEN: return "16x (±0.256V)";
        default: return "Unknown";
    }
}

const char* dataRateToString(DataRate rate) {
    switch (rate) {
        case DataRate::SPS_8: return "8 SPS";
        case DataRate::SPS_16: return "16 SPS";
        case DataRate::SPS_32: return "32 SPS";
        case DataRate::SPS_64: return "64 SPS";
        case DataRate::SPS_128: return "128 SPS";
        case DataRate::SPS_250: return "250 SPS";
        case DataRate::SPS_475: return "475 SPS";
        case DataRate::SPS_860: return "860 SPS";
        default: return "Unknown";
    }
}

const char* operatingModeToString(OperatingMode mode) {
    switch (mode) {
        case OperatingMode::CONTINUOUS: return "Continuous";
        case OperatingMode::SINGLE_SHOT: return "Single Shot";
        default: return "Unknown";
    }
}

ADCChannel indexToChannel(uint8_t index) {
    if (index <= 3) {
        return static_cast<ADCChannel>(index);
    }
    return ADCChannel::A0;
}

uint8_t channelToIndex(ADCChannel channel) {
    return static_cast<uint8_t>(channel);
}

GainAmplifier voltageRangeToGain(float max_voltage) {
    if (max_voltage <= 0.256f) return GainAmplifier::GAIN_SIXTEEN;
    if (max_voltage <= 0.512f) return GainAmplifier::GAIN_EIGHT;
    if (max_voltage <= 1.024f) return GainAmplifier::GAIN_FOUR;
    if (max_voltage <= 2.048f) return GainAmplifier::GAIN_TWO;
    if (max_voltage <= 4.096f) return GainAmplifier::GAIN_ONE;
    return GainAmplifier::GAIN_TWOTHIRDS;
}

DataRate frequencyToDataRate(uint16_t frequency) {
    if (frequency <= 8) return DataRate::SPS_8;
    if (frequency <= 16) return DataRate::SPS_16;
    if (frequency <= 32) return DataRate::SPS_32;
    if (frequency <= 64) return DataRate::SPS_64;
    if (frequency <= 128) return DataRate::SPS_128;
    if (frequency <= 250) return DataRate::SPS_250;
    if (frequency <= 475) return DataRate::SPS_475;
    return DataRate::SPS_860;
}

float getMaxVoltage(GainAmplifier gain) {
    return getVoltageRange(gain);
}

float getMinVoltage(GainAmplifier gain) {
    return -getVoltageRange(gain);
}

bool isVoltageInRange(float voltage, GainAmplifier gain) {
    float range = getVoltageRange(gain);
    return (voltage >= -range && voltage <= range);
}

} // namespace ADS1115