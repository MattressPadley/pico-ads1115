#include "ads1115/ads1115.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

using namespace ADS1115;

// Configuration
constexpr uint8_t ADS1115_ADDRESS = DEFAULT_I2C_ADDRESS;  // 0x48
constexpr uint SDA_PIN = PICO_DEFAULT_I2C_SDA_PIN;        // GPIO 4
constexpr uint SCL_PIN = PICO_DEFAULT_I2C_SCL_PIN;        // GPIO 5
constexpr uint BAUDRATE = 100000;                         // 100 kHz I2C

// Function prototypes
void setup_hardware();
void i2c_scan();
void demonstrate_basic_reading();
void demonstrate_all_channels();
void demonstrate_differential_reading();
void demonstrate_continuous_mode();
void demonstrate_comparator();
void demonstrate_external_configuration();
void print_system_info();

// Global ADS1115 device instance (traditional method)
ADS1115Device adc(i2c_default, ADS1115_ADDRESS);

// Alternative: Using DeviceConfig for external configuration
DeviceConfig device_config;
ADS1115Device adc_configured(device_config);

int main() {
    // Initialize hardware
    setup_hardware();
    
    // Wait for serial connection
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000); // Give time for terminal to connect
    
    printf("\n=== ADS1115 Basic ADC Example ===\n");
    print_system_info();
    
    // Initialize I2C hardware (app responsibility, not library)
    printf("\nInitializing I2C hardware...\n");
    i2c_init(i2c_default, BAUDRATE);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    printf("I2C initialized on pins %d (SDA) and %d (SCL) at %d Hz\n", SDA_PIN, SCL_PIN, BAUDRATE);
    
    // Perform I2C scan
    i2c_scan();
    
    // Test direct I2C communication first
    printf("\nTesting direct I2C communication with ADS1115...\n");
    uint8_t test_data[2];
    int ret = i2c_read_blocking(i2c_default, ADS1115_ADDRESS, test_data, 2, false);
    if (ret >= 0) {
        printf("Direct I2C read successful: 0x%02X 0x%02X\n", test_data[0], test_data[1]);
    } else {
        printf("Direct I2C read failed: %d\n", ret);
    }
    
    // Initialize the ADS1115 device (I2C already initialized above)
    printf("\nInitializing ADS1115 device...\n");
    Error err = adc.begin();
    if (err != Error::SUCCESS) {
        printf("Failed to initialize ADS1115: %s\n", adc.getErrorString(err));
        return 1;
    }
    
    printf("ADS1115 initialized successfully!\n");
    adc.printConfiguration();
    
    // Run demonstrations
    while (true) {
        printf("\n==================================================\n");
        printf("Choose a demonstration:\n");
        printf("1. Basic single channel reading\n");
        printf("2. Read all channels\n");
        printf("3. Differential reading\n");
        printf("4. Continuous mode\n");
        printf("5. Comparator demo\n");
        printf("6. External configuration demo\n");
        printf("7. Print device status\n");
        printf("0. Run all demos in sequence\n");
        printf("Press any key to continue...\n");
        
        // Wait for input (simplified - in real application you'd handle input properly)
        getchar();
        
        // For this example, we'll cycle through all demos
        static int demo_counter = 0;
        switch (demo_counter % 7) {
            case 0:
                demonstrate_basic_reading();
                break;
            case 1:
                demonstrate_all_channels();
                break;
            case 2:
                demonstrate_differential_reading();
                break;
            case 3:
                demonstrate_continuous_mode();
                break;
            case 4:
                demonstrate_comparator();
                break;
            case 5:
                demonstrate_external_configuration();
                break;
            case 6:
                adc.printStatus();
                printf("\n");
                adc.printConfiguration();
                break;
        }
        
        demo_counter++;
        sleep_ms(3000);  // Wait 3 seconds between demos
    }
    
    return 0;
}

void setup_hardware() {
    // Initialize stdio for USB/UART output
    stdio_init_all();
    
    printf("Hardware initialized\n");
}

void demonstrate_basic_reading() {
    printf("\n--- Basic Single Channel Reading ---\n");
    
    // Read from channel A0
    ADCReading reading;
    Error err = adc.readChannel(ADCChannel::A0, reading);
    
    if (err == Error::SUCCESS) {
        printf("Channel A0 Reading:\n");
        printf("  Raw Value: %d\n", reading.raw_value);
        printf("  Voltage: %.6f V\n", reading.voltage);
        printf("  Valid: %s\n", reading.valid ? "Yes" : "No");
    } else {
        printf("Failed to read channel A0: %s\n", adc.getErrorString(err));
    }
    
    // Demonstrate different gain settings
    printf("\nTesting different gain settings on A0:\n");
    GainAmplifier gains[] = {
        GainAmplifier::GAIN_TWOTHIRDS,
        GainAmplifier::GAIN_ONE,
        GainAmplifier::GAIN_TWO,
        GainAmplifier::GAIN_FOUR
    };
    
    for (auto gain : gains) {
        adc.updateGain(gain);
        sleep_ms(100);  // Allow settling time
        
        err = adc.readChannel(ADCChannel::A0, reading);
        if (err == Error::SUCCESS) {
            printf("  Gain %s: %.6f V (Raw: %d)\n", 
                   gainToString(gain), reading.voltage, reading.raw_value);
        }
    }
    
    // Restore default gain
    adc.updateGain(GainAmplifier::GAIN_TWO);
}

void demonstrate_all_channels() {
    printf("\n--- Reading All Channels ---\n");
    
    ADCReading readings[4];
    Error err = adc.readAllChannels(readings);
    
    if (err == Error::SUCCESS) {
        printf("All Channel Readings:\n");
        for (int i = 0; i < 4; i++) {
            printf("  A%d: %.6f V (Raw: %d)\n", 
                   i, readings[i].voltage, readings[i].raw_value);
        }
    } else {
        printf("Failed to read all channels: %s\n", adc.getErrorString(err));
    }
    
    // Individual channel readings with timing
    printf("\nIndividual channel readings with timing:\n");
    for (int i = 0; i < 4; i++) {
        ADCChannel channel = static_cast<ADCChannel>(i);
        ADCReading reading;
        
        absolute_time_t start_time = get_absolute_time();
        err = adc.readChannel(channel, reading);
        absolute_time_t end_time = get_absolute_time();
        
        uint32_t duration_us = absolute_time_diff_us(start_time, end_time);
        
        if (err == Error::SUCCESS) {
            printf("  %s: %.6f V (Raw: %d) - %lu µs\n", 
                   channelToString(channel), reading.voltage, 
                   reading.raw_value, duration_us);
        }
    }
}

void demonstrate_differential_reading() {
    printf("\n--- Differential Reading ---\n");
    
    // Test differential channels
    ADCChannel diff_channels[] = {
        ADCChannel::DIFFERENTIAL_0_1,
        ADCChannel::DIFFERENTIAL_0_3,
        ADCChannel::DIFFERENTIAL_1_3,
        ADCChannel::DIFFERENTIAL_2_3
    };
    
    printf("Differential readings:\n");
    for (auto channel : diff_channels) {
        ADCReading reading;
        Error err = adc.readDifferential(channel, reading);
        
        if (err == Error::SUCCESS) {
            printf("  %s: %.6f V (Raw: %d)\n", 
                   channelToString(channel), reading.voltage, reading.raw_value);
        } else {
            printf("  %s: Error - %s\n", 
                   channelToString(channel), adc.getErrorString(err));
        }
    }
}

void demonstrate_continuous_mode() {
    printf("\n--- Continuous Mode Reading ---\n");
    
    // Start continuous mode on channel A0
    Error err = adc.startContinuousMode(ADCChannel::A0);
    if (err != Error::SUCCESS) {
        printf("Failed to start continuous mode: %s\n", adc.getErrorString(err));
        return;
    }
    
    printf("Continuous mode started on A0. Reading 10 samples...\n");
    
    // Read 10 samples
    for (int i = 0; i < 10; i++) {
        ADCReading reading;
        err = adc.readContinuous(reading);
        
        if (err == Error::SUCCESS) {
            printf("  Sample %d: %.6f V (Raw: %d)\n", 
                   i + 1, reading.voltage, reading.raw_value);
        } else {
            printf("  Sample %d: Error - %s\n", 
                   i + 1, adc.getErrorString(err));
        }
        
        sleep_ms(100);  // 100ms between samples
    }
    
    // Stop continuous mode
    err = adc.stopContinuousMode();
    if (err == Error::SUCCESS) {
        printf("Continuous mode stopped.\n");
    } else {
        printf("Failed to stop continuous mode: %s\n", adc.getErrorString(err));
    }
}

void demonstrate_comparator() {
    printf("\n--- Comparator Demonstration ---\n");
    
    // Set up comparator thresholds
    float low_threshold = 1.0f;   // 1.0V
    float high_threshold = 3.0f;  // 3.0V
    
    printf("Setting up comparator:\n");
    printf("  Low threshold: %.3f V\n", low_threshold);
    printf("  High threshold: %.3f V\n", high_threshold);
    
    Error err = adc.updateThresholds(low_threshold, high_threshold);
    if (err != Error::SUCCESS) {
        printf("Failed to set thresholds: %s\n", adc.getErrorString(err));
        return;
    }
    
    // Enable comparator
    err = adc.enableComparator(ComparatorQueue::ASSERT_AFTER_ONE);
    if (err != Error::SUCCESS) {
        printf("Failed to enable comparator: %s\n", adc.getErrorString(err));
        return;
    }
    
    printf("Comparator enabled. Reading A0 and checking alerts...\n");
    
    // Read and check alerts
    for (int i = 0; i < 5; i++) {
        ADCReading reading;
        err = adc.readChannel(ADCChannel::A0, reading);
        
        if (err == Error::SUCCESS) {
            bool alert_active = adc.isAlertActive();
            printf("  Reading %d: %.6f V - Alert: %s\n", 
                   i + 1, reading.voltage, alert_active ? "ACTIVE" : "Inactive");
            
            if (alert_active) {
                printf("    Alert triggered! Voltage outside thresholds.\n");
                adc.clearAlert();
            }
        }
        
        sleep_ms(500);
    }
    
    // Disable comparator
    err = adc.disableComparator();
    if (err == Error::SUCCESS) {
        printf("Comparator disabled.\n");
    }
}

// I2C scan function
void i2c_scan() {
    printf("\nScanning I2C bus for devices...\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        
        // Skip reserved addresses
        if ((addr & 0x78) == 0 || (addr & 0x78) == 0x78) {
            printf("   ");
        } else {
            uint8_t rxdata;
            int ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);
            if (ret >= 0) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        
        if (addr % 16 == 15) {
            printf("\n");
        }
    }
    printf("I2C scan complete.\n\n");
}

void print_system_info() {
    printf("\nSystem Information:\n");
    printf("  SDK Version: %s\n", PICO_SDK_VERSION_STRING);
    printf("  I2C Instance: %s\n", i2c_default == i2c0 ? "i2c0" : "i2c1");
    printf("  SDA Pin: %d\n", SDA_PIN);
    printf("  SCL Pin: %d\n", SCL_PIN);
    printf("  Baudrate: %d Hz\n", BAUDRATE);
    printf("  ADS1115 Address: 0x%02X\n", ADS1115_ADDRESS);
}

void demonstrate_external_configuration() {
    printf("\n--- External Configuration Demonstration ---\n");
    
    // Create a custom device configuration
    DeviceConfig custom_config;
    custom_config.i2c_instance = i2c_default;
    custom_config.device_address = 0x48;
    custom_config.i2c_baudrate = 400000;  // 400kHz instead of default 100kHz
    custom_config.sda_pin = PICO_DEFAULT_I2C_SDA_PIN;
    custom_config.scl_pin = PICO_DEFAULT_I2C_SCL_PIN;
    custom_config.enable_pullups = true;
    custom_config.alert_pin = 22;  // Use GPIO 22 for alert
    custom_config.alert_enabled = true;
    custom_config.power_on_delay_ms = 50;  // Longer power-on delay
    custom_config.auto_init_i2c = false;  // Don't auto-initialize I2C (already done)
    custom_config.validate_connections = true;
    
    // Configure default ADC settings
    custom_config.default_adc_config.gain = GainAmplifier::GAIN_FOUR;  // ±1.024V range
    custom_config.default_adc_config.data_rate = DataRate::SPS_250;     // 250 SPS
    custom_config.default_adc_config.mode = OperatingMode::SINGLE_SHOT;
    
    printf("Custom configuration:\n");
    printf("  I2C Baudrate: %lu Hz\n", custom_config.i2c_baudrate);
    printf("  Device Address: 0x%02X\n", custom_config.device_address);
    printf("  Alert Pin: %d\n", custom_config.alert_pin);
    printf("  Power-on Delay: %d ms\n", custom_config.power_on_delay_ms);
    printf("  Default Gain: %s\n", gainToString(custom_config.default_adc_config.gain));
    printf("  Default Data Rate: %s\n", dataRateToString(custom_config.default_adc_config.data_rate));
    
    // Create device with custom configuration
    ADS1115Device custom_adc(custom_config);
    
    printf("\nInitializing device with custom configuration...\n");
    Error err = custom_adc.begin(custom_config);
    
    if (err == Error::SUCCESS) {
        printf("Custom device initialized successfully!\n");
        
        // Demonstrate reading with custom configuration
        printf("\nReading with custom configuration:\n");
        ADCReading reading;
        err = custom_adc.readChannel(ADCChannel::A0, reading);
        
        if (err == Error::SUCCESS) {
            printf("  Channel A0: %.6f V (Raw: %d)\n", reading.voltage, reading.raw_value);
            printf("  Gain: %s\n", gainToString(custom_adc.getGain()));
            printf("  Data Rate: %s\n", dataRateToString(custom_adc.getDataRate()));
        } else {
            printf("  Failed to read: %s\n", custom_adc.getErrorString(err));
        }
        
        // Demonstrate runtime configuration updates
        printf("\nDemonstrating runtime configuration updates:\n");
        printf("  Updating gain to GAIN_EIGHT...\n");
        custom_adc.updateGain(GainAmplifier::GAIN_EIGHT);
        
        printf("  Updating data rate to SPS_860...\n");
        custom_adc.updateDataRate(DataRate::SPS_860);
        
        err = custom_adc.readChannel(ADCChannel::A0, reading);
        if (err == Error::SUCCESS) {
            printf("  Updated reading: %.6f V (Raw: %d)\n", reading.voltage, reading.raw_value);
            printf("  New Gain: %s\n", gainToString(custom_adc.getGain()));
            printf("  New Data Rate: %s\n", dataRateToString(custom_adc.getDataRate()));
        }
        
        printf("\nExternal configuration demonstration complete.\n");
    } else {
        printf("Failed to initialize custom device: %s\n", custom_adc.getErrorString(err));
    }
}