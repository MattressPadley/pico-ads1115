# ADS1115 Library for Raspberry Pi Pico

A comprehensive C++ library for the ADS1115 16-bit ADC with I2C interface, designed for the Raspberry Pi Pico using the **Pre-Initialized I2C Pattern** for robust multi-device support.

## Features

- **Pre-Initialized I2C Pattern**: Designed for multi-device I2C scenarios
- **External Hardware Configuration**: Configure I2C, pins, and settings outside the library
- **16-bit precision ADC** with 4 input channels
- **Programmable gain amplifier** (±6.144V to ±0.256V ranges)
- **Flexible input configuration** (single-ended or differential)
- **Multiple operating modes** (single-shot and continuous)
- **Configurable data rates** (8 to 860 samples per second)
- **Comparator functionality** with alert/interrupt support
- **Runtime configuration updates** with intuitive `update*()` methods
- **Comprehensive error handling**
- **Multi-device I2C support** without conflicts
- **Thread-safe design**

## Hardware Requirements

- Raspberry Pi Pico or compatible RP2040 board
- ADS1115 ADC module
- I2C connection (SDA, SCL)
- Optional: Alert/Ready pin connection

## Wiring

| ADS1115 Pin | Pico Pin | Description |
|-------------|----------|-------------|
| VDD         | 3.3V     | Power supply |
| GND         | GND      | Ground |
| SCL         | GP5      | I2C Clock (configurable) |
| SDA         | GP4      | I2C Data (configurable) |
| ADDR        | GND      | I2C Address selection |
| ALRT        | GP2      | Alert/Ready pin (optional) |
| A0-A3       | -        | Analog input channels |

## Installation

1. Clone this repository into your Pico project directory
2. Add the library to your CMakeLists.txt:

```cmake
add_subdirectory(pico-ads1115)
target_link_libraries(your_target ads1115)
```

3. Include the header in your code:

```cpp
#include "ads1115/ads1115.hpp"
```

## Pre-Initialized I2C Pattern

This library implements the **Pre-Initialized I2C Pattern** for better multi-device support:

### The Problem with Traditional Libraries
```cpp
// PROBLEMATIC: Each library manages I2C
ADS1115Device adc1(i2c0, 0x48, 4, 5);  // Initializes I2C
ADS1115Device adc2(i2c0, 0x49, 4, 5);  // Re-initializes I2C
adc1.begin(100000);  // Sets baudrate
adc2.begin(400000);  // Changes baudrate (breaks adc1!)
```

### The Solution: Pre-Initialized I2C
```cpp
// CORRECT: App controls I2C, libraries handle devices
i2c_init(i2c_default, 100000);
gpio_set_function(4, GPIO_FUNC_I2C);
gpio_set_function(5, GPIO_FUNC_I2C);
gpio_pull_up(4);
gpio_pull_up(5);

// Create devices (no pin parameters)
ADS1115Device adc1(i2c_default, 0x48);
ADS1115Device adc2(i2c_default, 0x49);

// Initialize devices (no baudrate conflicts)
adc1.begin();
adc2.begin();
```

## External Hardware Configuration

In addition to the Pre-Initialized I2C Pattern, this library supports **External Hardware Configuration** using the `DeviceConfig` structure. This allows you to configure all hardware settings outside the library for maximum flexibility.

### Traditional vs. External Configuration

**Traditional approach (still supported):**
```cpp
// App handles I2C initialization
i2c_init(i2c_default, 100000);
gpio_set_function(4, GPIO_FUNC_I2C);
gpio_set_function(5, GPIO_FUNC_I2C);
gpio_pull_up(4);
gpio_pull_up(5);

// Create device with minimal configuration
ADS1115Device adc(i2c_default, 0x48);
adc.begin();
```

**External configuration approach (new):**
```cpp
#include "ads1115/ads1115_config.hpp"

// Configure everything externally
DeviceConfig config;
config.i2c_instance = i2c0;
config.device_address = 0x48;
config.i2c_baudrate = 400000;           // 400kHz I2C
config.sda_pin = 4;
config.scl_pin = 5;
config.enable_pullups = true;
config.alert_pin = 22;                  // Use GPIO 22 for alerts
config.alert_enabled = true;
config.auto_init_i2c = true;            // Library handles I2C init
config.validate_connections = true;
config.power_on_delay_ms = 50;          // Custom power-on delay

// Configure default ADC settings
config.default_adc_config.gain = GainAmplifier::GAIN_FOUR;
config.default_adc_config.data_rate = DataRate::SPS_250;
config.default_adc_config.mode = OperatingMode::SINGLE_SHOT;

// Create and initialize with external configuration
ADS1115Device adc(config);
Error err = adc.begin(config);          // All hardware configured automatically
```

### DeviceConfig Structure

The `DeviceConfig` structure provides comprehensive hardware configuration:

```cpp
struct DeviceConfig {
    // I2C Hardware Configuration
    i2c_inst_t* i2c_instance = nullptr;        // I2C instance (i2c0, i2c1)
    uint8_t device_address = 0x48;              // I2C device address
    uint32_t i2c_baudrate = 100000;             // I2C clock speed in Hz
    uint sda_pin = PICO_DEFAULT_I2C_SDA_PIN;    // SDA pin number
    uint scl_pin = PICO_DEFAULT_I2C_SCL_PIN;    // SCL pin number
    bool enable_pullups = true;                  // Enable internal pull-up resistors
    
    // Alert/Interrupt Configuration
    uint alert_pin = 255;                       // Alert pin (255 = disabled)
    bool alert_enabled = false;                 // Enable alert functionality
    
    // Power Management
    uint16_t power_on_delay_ms = 25;            // Power-on delay in milliseconds
    uint16_t reset_delay_ms = 10;               // Reset stabilization delay in milliseconds
    
    // Default ADC Configuration
    ADCConfig default_adc_config = ADCConfig(); // Default ADC settings
    
    // I2C Communication Settings
    uint8_t i2c_timeout_ms = 100;               // I2C timeout in milliseconds
    uint8_t max_retries = 3;                    // Maximum I2C retry attempts
    
    // Validation flags
    bool auto_init_i2c = false;                 // Automatically initialize I2C hardware
    bool validate_connections = true;            // Validate I2C connections on begin()
};
```

### Configuration Examples

**High-speed I2C with alert:**
```cpp
DeviceConfig high_speed_config;
high_speed_config.i2c_instance = i2c1;
high_speed_config.i2c_baudrate = 1000000;    // 1MHz I2C
high_speed_config.sda_pin = 6;
high_speed_config.scl_pin = 7;
high_speed_config.alert_pin = 15;
high_speed_config.alert_enabled = true;
high_speed_config.auto_init_i2c = true;
```

**Multiple devices with different settings:**
```cpp
// Fast ADC for data acquisition
DeviceConfig fast_adc_config;
fast_adc_config.i2c_instance = i2c0;
fast_adc_config.device_address = 0x48;
fast_adc_config.i2c_baudrate = 400000;
fast_adc_config.default_adc_config.data_rate = DataRate::SPS_860;
fast_adc_config.auto_init_i2c = true;

// Precise ADC for measurements
DeviceConfig precise_adc_config;
precise_adc_config.i2c_instance = i2c0;
precise_adc_config.device_address = 0x49;
precise_adc_config.i2c_baudrate = 100000;    // Slower for precision
precise_adc_config.default_adc_config.data_rate = DataRate::SPS_8;
precise_adc_config.default_adc_config.gain = GainAmplifier::GAIN_SIXTEEN;
precise_adc_config.power_on_delay_ms = 100;  // Longer stabilization

ADS1115Device fast_adc(fast_adc_config);
ADS1115Device precise_adc(precise_adc_config);

fast_adc.begin(fast_adc_config);
precise_adc.begin(precise_adc_config);
```

## Quick Start

```cpp
#include "ads1115/ads1115.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
using namespace ADS1115;

int main() {
    stdio_init_all();
    
    // Initialize I2C hardware (app responsibility)
    i2c_init(i2c_default, 100000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
    // Create and initialize ADS1115 (no I2C parameters)
    ADS1115Device adc(i2c_default, 0x48);
    if (adc.begin() != Error::SUCCESS) {
        printf("Failed to initialize ADS1115\n");
        return 1;
    }
    
    // Read a single channel
    ADCReading reading;
    if (adc.readChannel(ADCChannel::A0, reading) == Error::SUCCESS) {
        printf("A0: %.3fV (raw: %d)\n", reading.voltage, reading.raw_value);
    }
    
    return 0;
}
```

## Multi-Device Example

```cpp
// Initialize I2C once for all devices
i2c_init(i2c_default, 100000);
gpio_set_function(4, GPIO_FUNC_I2C);
gpio_set_function(5, GPIO_FUNC_I2C);
gpio_pull_up(4);
gpio_pull_up(5);

// Create multiple devices on same bus
ADS1115Device adc1(i2c_default, 0x48);  // ADDR to GND
ADS1115Device adc2(i2c_default, 0x49);  // ADDR to VDD
CAP1188Device touch(i2c_default, 0x29); // Touch sensor

// Initialize all devices (no conflicts)
adc1.begin();
adc2.begin();
touch.begin();
```

## API Reference

### Core Classes

#### `ADS1115Device`
Main class for interfacing with the ADS1115.

**Constructors (v2.x - Pre-Initialized I2C Pattern):**
```cpp
// Traditional constructor
ADS1115Device(i2c_inst_t* i2c_instance, 
              uint8_t device_address = DEFAULT_I2C_ADDRESS,
              uint alert_pin = 255);

// External configuration constructor (v2.1+)
ADS1115Device(const DeviceConfig& config);
```

**Migration from v1.x:**
```cpp
// Before (v1.x)
ADS1115Device adc(i2c0, 0x48, 4, 5, 255);  // Pins in constructor
adc.begin(100000);                          // Baudrate in begin()

// After (v2.x)
i2c_init(i2c_default, 100000);              // App handles I2C
gpio_set_function(4, GPIO_FUNC_I2C);
gpio_set_function(5, GPIO_FUNC_I2C);
gpio_pull_up(4);
gpio_pull_up(5);

ADS1115Device adc(i2c_default, 0x48);       // No pins in constructor
adc.begin();                                // No baudrate in begin()
```

### Key Methods

#### Device Management
- `Error begin()` - Initialize the device (I2C must be pre-initialized)
- `Error begin(const DeviceConfig& config)` - Initialize with external configuration
- `bool isConnected()` - Check if device is responding
- `Error reset()` - Reset the device to default settings
- `DeviceStatus getStatus()` - Get current device status

#### ADC Reading
- `Error readChannel(ADCChannel channel, ADCReading& reading)` - Read a single channel
- `Error readChannelVoltage(ADCChannel channel, float& voltage)` - Read voltage directly
- `Error readAllChannels(ADCReading readings[4])` - Read all 4 channels
- `Error readDifferential(ADCChannel diff_channel, ADCReading& reading)` - Read differential pair

#### Configuration (Runtime Updates)
- `Error updateGain(GainAmplifier gain)` - Update programmable gain amplifier
- `Error updateDataRate(DataRate rate)` - Update sampling rate
- `Error updateOperatingMode(OperatingMode mode)` - Update continuous or single-shot mode
- `Error updateConfiguration(const ADCConfig& config)` - Update entire ADC configuration

#### Continuous Mode
- `Error startContinuousMode(ADCChannel channel)` - Start continuous conversions
- `Error stopContinuousMode()` - Stop continuous mode
- `Error readContinuous(ADCReading& reading)` - Read from continuous mode

#### Comparator/Alert
- `Error updateThresholds(float low, float high)` - Update alert thresholds
- `Error updateThresholds(const ThresholdConfig& config)` - Update thresholds with config
- `Error enableComparator(ComparatorQueue queue)` - Enable comparator
- `Error updateComparator(ComparatorMode mode, ComparatorPolarity polarity)` - Update comparator settings
- `bool isAlertActive()` - Check if alert is active
- `Error clearAlert()` - Clear alert condition

#### Callbacks
- `void updateConversionCallback(ConversionCallback callback)` - Set conversion complete callback
- `void updateAlertCallback(AlertCallback callback)` - Set alert callback
- `void updateErrorCallback(ErrorCallback callback)` - Set error callback

### Enums and Types

#### `ADCChannel`
```cpp
enum class ADCChannel {
    A0, A1, A2, A3,                    // Single-ended channels
    DIFFERENTIAL_0_1, DIFFERENTIAL_0_3, // Differential channels
    DIFFERENTIAL_1_3, DIFFERENTIAL_2_3
};
```

#### `GainAmplifier`
```cpp
enum class GainAmplifier {
    GAIN_TWOTHIRDS,  // ±6.144V
    GAIN_ONE,        // ±4.096V
    GAIN_TWO,        // ±2.048V (default)
    GAIN_FOUR,       // ±1.024V
    GAIN_EIGHT,      // ±0.512V
    GAIN_SIXTEEN     // ±0.256V
};
```

#### `DataRate`
```cpp
enum class DataRate {
    SPS_8, SPS_16, SPS_32, SPS_64,
    SPS_128,  // Default
    SPS_250, SPS_475, SPS_860
};
```

#### `ADCReading`
```cpp
struct ADCReading {
    int16_t raw_value;      // Raw ADC value
    float voltage;          // Converted voltage
    ADCChannel channel;     // Channel read
    uint32_t timestamp;     // Reading timestamp
    bool valid;             // Reading validity
};
```

#### `DeviceConfig` (v2.1+)
```cpp
struct DeviceConfig {
    // I2C Hardware Configuration
    i2c_inst_t* i2c_instance = nullptr;
    uint8_t device_address = 0x48;
    uint32_t i2c_baudrate = 100000;
    uint sda_pin = PICO_DEFAULT_I2C_SDA_PIN;
    uint scl_pin = PICO_DEFAULT_I2C_SCL_PIN;
    bool enable_pullups = true;
    
    // Alert/Interrupt Configuration
    uint alert_pin = 255;              // 255 = disabled
    bool alert_enabled = false;
    
    // Power Management
    uint16_t power_on_delay_ms = 25;
    uint16_t reset_delay_ms = 10;
    
    // Default ADC Configuration
    ADCConfig default_adc_config = ADCConfig();
    
    // Validation flags
    bool auto_init_i2c = false;        // Auto-initialize I2C hardware
    bool validate_connections = true;  // Validate I2C on begin()
};
```

## Examples

### Basic Channel Reading
```cpp
ADCReading reading;
Error err = adc.readChannel(ADCChannel::A0, reading);
if (err == Error::SUCCESS) {
    printf("A0: %.6f V\n", reading.voltage);
}
```

### Differential Reading
```cpp
ADCReading diff_reading;
Error err = adc.readDifferential(ADCChannel::DIFFERENTIAL_0_1, diff_reading);
if (err == Error::SUCCESS) {
    printf("Differential 0-1: %.6f V\n", diff_reading.voltage);
}
```

### Continuous Mode
```cpp
// Start continuous mode
adc.startContinuousMode(ADCChannel::A0);

// Read continuously
for (int i = 0; i < 10; i++) {
    ADCReading reading;
    if (adc.readContinuous(reading) == Error::SUCCESS) {
        printf("Sample %d: %.6f V\n", i, reading.voltage);
    }
    sleep_ms(100);
}

// Stop continuous mode
adc.stopContinuousMode();
```

### Comparator/Alert
```cpp
// Update thresholds
adc.updateThresholds(1.0f, 3.0f);  // Low: 1V, High: 3V

// Enable comparator
adc.enableComparator(ComparatorQueue::ASSERT_AFTER_ONE);

// Check for alerts
ADCReading reading;
adc.readChannel(ADCChannel::A0, reading);
if (adc.isAlertActive()) {
    printf("Alert! Voltage outside thresholds: %.6f V\n", reading.voltage);
    adc.clearAlert();
}
```

### External Configuration Example
```cpp
#include "ads1115/ads1115_config.hpp"

// Create configuration for high-speed data acquisition
DeviceConfig fast_config;
fast_config.i2c_instance = i2c0;
fast_config.device_address = 0x48;
fast_config.i2c_baudrate = 400000;                    // Fast I2C
fast_config.sda_pin = 4;
fast_config.scl_pin = 5;
fast_config.alert_pin = 22;
fast_config.alert_enabled = true;
fast_config.auto_init_i2c = true;                     // Auto-init I2C
fast_config.default_adc_config.data_rate = DataRate::SPS_860;  // Fastest sampling
fast_config.default_adc_config.gain = GainAmplifier::GAIN_TWO; // ±2.048V range

// Create and initialize device
ADS1115Device fast_adc(fast_config);
Error err = fast_adc.begin(fast_config);

if (err == Error::SUCCESS) {
    printf("Fast ADC initialized successfully!\n");
    
    // Runtime configuration updates
    fast_adc.updateGain(GainAmplifier::GAIN_FOUR);     // Switch to ±1.024V
    fast_adc.updateDataRate(DataRate::SPS_475);        // Reduce noise
    
    // Read with new settings
    ADCReading reading;
    fast_adc.readChannel(ADCChannel::A0, reading);
    printf("Updated reading: %.6f V\n", reading.voltage);
}
```

## Error Handling

All functions return an `Error` enum:

```cpp
enum class Error {
    SUCCESS,
    I2C_ERROR,
    DEVICE_NOT_FOUND,
    INVALID_PARAMETER,
    TIMEOUT,
    NOT_INITIALIZED,
    CONFIGURATION_ERROR,
    CONVERSION_ERROR
};
```

Use `adc.getErrorString(error)` to get human-readable error messages.

## I2C Addressing

The ADS1115 supports 4 I2C addresses based on the ADDR pin connection:

| ADDR Pin | I2C Address |
|----------|-------------|
| GND      | 0x48        |
| VDD      | 0x49        |
| SDA      | 0x4A        |
| SCL      | 0x4B        |

## Performance

- **Conversion time**: Depends on data rate (1.25ms at 860 SPS to 125ms at 8 SPS)
- **I2C overhead**: ~1ms per register read/write
- **Resolution**: 16-bit (15-bit + sign for single-ended)
- **Accuracy**: Depends on gain setting and reference voltage

## Troubleshooting

### Common Issues

1. **Device not found**: Check I2C wiring and address
2. **Incorrect readings**: Verify gain settings and voltage ranges
3. **I2C errors**: Check pull-up resistors and bus speed
4. **Alert not working**: Ensure alert pin is configured and thresholds are set

### Debug Functions

```cpp
adc.printStatus();         // Print device status
adc.printConfiguration();  // Print current configuration
adc.printReading(reading); // Print reading details
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Benefits of Pre-Initialized I2C Pattern

✅ **True Multi-Device Support** - No I2C conflicts between libraries  
✅ **Predictable Behavior** - App controls exactly when I2C is configured  
✅ **Easier Debugging** - Clear separation between hardware and device setup  
✅ **Better Performance** - No redundant I2C initialization  
✅ **Flexible Configuration** - App can optimize I2C settings for all devices  
✅ **Clean Architecture** - Libraries focus on device communication, not hardware management

## Version History

### v2.1.0 - External Hardware Configuration (Current)
- **NEW**: Added `DeviceConfig` structure for external hardware configuration
- **NEW**: Added `DeviceConfig` constructor for flexible initialization
- **NEW**: Added `begin(DeviceConfig)` method for external configuration
- **BREAKING**: Renamed all `set*()` methods to `update*()` for better semantics
  - `setGain()` → `updateGain()`
  - `setDataRate()` → `updateDataRate()`
  - `setOperatingMode()` → `updateOperatingMode()`
  - `setThresholds()` → `updateThresholds()`
  - `setConfiguration()` → `updateConfiguration()`
  - All callback methods now use `update*()` naming
- **NEW**: Added `ads1115_config.hpp` with configuration constants and validation
- **NEW**: Support for automatic I2C hardware initialization
- **NEW**: Configurable power management and timing settings
- **NEW**: Enhanced alert/interrupt pin configuration
- **NEW**: Built-in configuration validation
- Improved documentation with comprehensive examples
- Maintains full backward compatibility for existing constructors and `begin()`

### v2.0.0 - Pre-Initialized I2C Pattern
- **BREAKING**: Removed SDA/SCL pin parameters from constructor
- **BREAKING**: Removed baudrate parameter from `begin()` method
- Added support for multi-device I2C scenarios
- Fixed circular dependency bug in device initialization
- Improved error handling and validation
- Updated examples to demonstrate new pattern

### v1.x - Traditional Pattern (Deprecated)
- Library managed I2C initialization
- Single-device focused design
- Pin parameters in constructor

## Acknowledgments

- Based on the Texas Instruments ADS1115 datasheet
- Inspired by the Adafruit ADS1X15 library
- Follows the same architectural patterns as the CAP1188 library
- Pre-Initialized I2C Pattern developed for robust multi-device support