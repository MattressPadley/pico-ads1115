#include "ads1115/ads1115.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>

using namespace ADS1115;

// I2C configuration
#define I2C_INSTANCE i2c0
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
#define I2C_BAUDRATE 100000

// ADS1115 configuration
#define ADS1115_ADDRESS 0x48

// Function prototypes
void setup_hardware();
void demonstrate_basic_reading();
void demonstrate_all_channels();
void demonstrate_differential_reading();
void demonstrate_continuous_mode();
void demonstrate_comparator();
void print_system_info();

// Global ADS1115 device
ADS1115Device adc(I2C_INSTANCE, ADS1115_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN);

int main() {
    // Initialize hardware
    setup_hardware();
    
    printf("\n=== ADS1115 Basic ADC Example ===\n");
    print_system_info();
    
    // Initialize the ADS1115
    printf("\nInitializing ADS1115...\n");
    Error err = adc.begin(I2C_BAUDRATE);
    if (err != Error::SUCCESS) {
        printf("Failed to initialize ADS1115: %s\n", adc.getErrorString(err));
        return 1;
    }
    
    printf("ADS1115 initialized successfully!\n");
    adc.printConfiguration();
    
    // Run demonstrations
    while (true) {
        printf("\n" "=".repeat(50) "\n");
        printf("Choose a demonstration:\n");
        printf("1. Basic single channel reading\n");
        printf("2. Read all channels\n");
        printf("3. Differential reading\n");
        printf("4. Continuous mode\n");
        printf("5. Comparator demo\n");
        printf("6. Print device status\n");
        printf("0. Run all demos in sequence\n");
        printf("Press any key to continue...\n");
        
        // Wait for input (simplified - in real application you'd handle input properly)
        getchar();
        
        // For this example, we'll cycle through all demos
        static int demo_counter = 0;
        switch (demo_counter % 6) {
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
    
    // Wait for USB connection (optional)
    sleep_ms(2000);
    
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
        adc.setGain(gain);
        sleep_ms(100);  // Allow settling time
        
        err = adc.readChannel(ADCChannel::A0, reading);
        if (err == Error::SUCCESS) {
            printf("  Gain %s: %.6f V (Raw: %d)\n", 
                   gainToString(gain), reading.voltage, reading.raw_value);
        }
    }
    
    // Restore default gain
    adc.setGain(GainAmplifier::GAIN_TWO);
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
    
    Error err = adc.setThresholds(low_threshold, high_threshold);
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

void print_system_info() {
    printf("\nSystem Information:\n");
    printf("  SDK Version: %s\n", PICO_SDK_VERSION_STRING);
    printf("  I2C Instance: %s\n", I2C_INSTANCE == i2c0 ? "i2c0" : "i2c1");
    printf("  SDA Pin: %d\n", I2C_SDA_PIN);
    printf("  SCL Pin: %d\n", I2C_SCL_PIN);
    printf("  Baudrate: %d Hz\n", I2C_BAUDRATE);
    printf("  ADS1115 Address: 0x%02X\n", ADS1115_ADDRESS);
}