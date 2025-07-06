#pragma once

#include <cstdint>

namespace ADS1115 {

// Default I2C address
constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x48;

// Available I2C addresses
constexpr uint8_t I2C_ADDRESS_0x48 = 0x48;  // ADDR connected to GND
constexpr uint8_t I2C_ADDRESS_0x49 = 0x49;  // ADDR connected to VDD
constexpr uint8_t I2C_ADDRESS_0x4A = 0x4A;  // ADDR connected to SDA
constexpr uint8_t I2C_ADDRESS_0x4B = 0x4B;  // ADDR connected to SCL

// Register addresses
constexpr uint8_t REG_CONVERSION = 0x00;     // Conversion result register
constexpr uint8_t REG_CONFIG = 0x01;         // Configuration register
constexpr uint8_t REG_LO_THRESH = 0x02;      // Low threshold register
constexpr uint8_t REG_HI_THRESH = 0x03;      // High threshold register

// Configuration register bit definitions
// Operational status/single-shot conversion start (bit 15)
constexpr uint16_t CONFIG_OS_MASK = 0x8000;
constexpr uint16_t CONFIG_OS_SINGLE = 0x8000;      // Start single conversion
constexpr uint16_t CONFIG_OS_BUSY = 0x0000;        // Device is performing conversion
constexpr uint16_t CONFIG_OS_NOT_BUSY = 0x8000;    // Device is not performing conversion

// Input multiplexer configuration (bits 14:12)
constexpr uint16_t CONFIG_MUX_MASK = 0x7000;
constexpr uint16_t CONFIG_MUX_DIFF_0_1 = 0x0000;   // Differential P = AIN0, N = AIN1
constexpr uint16_t CONFIG_MUX_DIFF_0_3 = 0x1000;   // Differential P = AIN0, N = AIN3
constexpr uint16_t CONFIG_MUX_DIFF_1_3 = 0x2000;   // Differential P = AIN1, N = AIN3
constexpr uint16_t CONFIG_MUX_DIFF_2_3 = 0x3000;   // Differential P = AIN2, N = AIN3
constexpr uint16_t CONFIG_MUX_SINGLE_0 = 0x4000;   // Single-ended AIN0
constexpr uint16_t CONFIG_MUX_SINGLE_1 = 0x5000;   // Single-ended AIN1
constexpr uint16_t CONFIG_MUX_SINGLE_2 = 0x6000;   // Single-ended AIN2
constexpr uint16_t CONFIG_MUX_SINGLE_3 = 0x7000;   // Single-ended AIN3

// Programmable gain amplifier configuration (bits 11:9)
constexpr uint16_t CONFIG_PGA_MASK = 0x0E00;
constexpr uint16_t CONFIG_PGA_6_144V = 0x0000;     // ±6.144V range
constexpr uint16_t CONFIG_PGA_4_096V = 0x0200;     // ±4.096V range
constexpr uint16_t CONFIG_PGA_2_048V = 0x0400;     // ±2.048V range (default)
constexpr uint16_t CONFIG_PGA_1_024V = 0x0600;     // ±1.024V range
constexpr uint16_t CONFIG_PGA_0_512V = 0x0800;     // ±0.512V range
constexpr uint16_t CONFIG_PGA_0_256V = 0x0A00;     // ±0.256V range

// Device operating mode (bit 8)
constexpr uint16_t CONFIG_MODE_MASK = 0x0100;
constexpr uint16_t CONFIG_MODE_CONTIN = 0x0000;     // Continuous conversion mode
constexpr uint16_t CONFIG_MODE_SINGLE = 0x0100;     // Single-shot conversion mode (default)

// Data rate (bits 7:5)
constexpr uint16_t CONFIG_DR_MASK = 0x00E0;
constexpr uint16_t CONFIG_DR_8SPS = 0x0000;         // 8 samples per second
constexpr uint16_t CONFIG_DR_16SPS = 0x0020;        // 16 samples per second
constexpr uint16_t CONFIG_DR_32SPS = 0x0040;        // 32 samples per second
constexpr uint16_t CONFIG_DR_64SPS = 0x0060;        // 64 samples per second
constexpr uint16_t CONFIG_DR_128SPS = 0x0080;       // 128 samples per second (default)
constexpr uint16_t CONFIG_DR_250SPS = 0x00A0;       // 250 samples per second
constexpr uint16_t CONFIG_DR_475SPS = 0x00C0;       // 475 samples per second
constexpr uint16_t CONFIG_DR_860SPS = 0x00E0;       // 860 samples per second

// Comparator mode (bit 4)
constexpr uint16_t CONFIG_CMODE_MASK = 0x0010;
constexpr uint16_t CONFIG_CMODE_TRAD = 0x0000;      // Traditional comparator (default)
constexpr uint16_t CONFIG_CMODE_WINDOW = 0x0010;    // Window comparator

// Comparator polarity (bit 3)
constexpr uint16_t CONFIG_CPOL_MASK = 0x0008;
constexpr uint16_t CONFIG_CPOL_ACTVLOW = 0x0000;    // Active low (default)
constexpr uint16_t CONFIG_CPOL_ACTVHI = 0x0008;     // Active high

// Comparator latching (bit 2)
constexpr uint16_t CONFIG_CLAT_MASK = 0x0004;
constexpr uint16_t CONFIG_CLAT_NONLAT = 0x0000;     // Non-latching (default)
constexpr uint16_t CONFIG_CLAT_LATCH = 0x0004;      // Latching

// Comparator queue and disable (bits 1:0)
constexpr uint16_t CONFIG_CQUE_MASK = 0x0003;
constexpr uint16_t CONFIG_CQUE_1CONV = 0x0000;      // Assert after one conversion
constexpr uint16_t CONFIG_CQUE_2CONV = 0x0001;      // Assert after two conversions
constexpr uint16_t CONFIG_CQUE_4CONV = 0x0002;      // Assert after four conversions
constexpr uint16_t CONFIG_CQUE_NONE = 0x0003;       // Disable comparator (default)

// Default configuration values
constexpr uint16_t DEFAULT_CONFIG = 
    CONFIG_OS_SINGLE |          // Start single conversion
    CONFIG_MUX_SINGLE_0 |       // Single-ended AIN0
    CONFIG_PGA_2_048V |         // ±2.048V range
    CONFIG_MODE_SINGLE |        // Single-shot mode
    CONFIG_DR_128SPS |          // 128 samples per second
    CONFIG_CMODE_TRAD |         // Traditional comparator
    CONFIG_CPOL_ACTVLOW |       // Active low
    CONFIG_CLAT_NONLAT |        // Non-latching
    CONFIG_CQUE_NONE;           // Disable comparator

// Default threshold values
constexpr int16_t DEFAULT_HIGH_THRESHOLD = 0x7FFF;  // Maximum positive value
constexpr int16_t DEFAULT_LOW_THRESHOLD = 0x8000;   // Maximum negative value

// Timing constants (in milliseconds)
constexpr uint16_t POWER_ON_DELAY_MS = 25;          // Power-on delay
constexpr uint16_t RESET_DELAY_MS = 10;             // Reset stabilization delay
constexpr uint8_t I2C_TIMEOUT_MS = 100;             // I2C timeout

// Maximum values
constexpr int16_t ADC_MAX_VALUE = 32767;             // Maximum ADC value
constexpr int16_t ADC_MIN_VALUE = -32768;            // Minimum ADC value
constexpr uint16_t ADC_RESOLUTION = 16;              // ADC resolution in bits
constexpr uint16_t ADC_MAX_UNSIGNED = 65535;        // Maximum unsigned value

// Conversion result register interpretation
constexpr uint8_t CONVERSION_RESULT_BITS = 16;      // Number of bits in conversion result
constexpr uint8_t CONVERSION_RESULT_SHIFT = 0;      // No shift needed for 16-bit result

// Configuration register reset value
constexpr uint16_t CONFIG_RESET_VALUE = 0x8583;     // Reset value of config register

// Helper macros for bit manipulation
#define SET_BITS(reg, mask, value) ((reg & ~mask) | (value & mask))
#define GET_BITS(reg, mask) (reg & mask)
#define CLEAR_BITS(reg, mask) (reg & ~mask)

// Channel to MUX mapping
constexpr uint16_t getChannelMux(uint8_t channel) {
    switch (channel) {
        case 0: return CONFIG_MUX_SINGLE_0;
        case 1: return CONFIG_MUX_SINGLE_1;
        case 2: return CONFIG_MUX_SINGLE_2;
        case 3: return CONFIG_MUX_SINGLE_3;
        case 4: return CONFIG_MUX_DIFF_0_1;
        case 5: return CONFIG_MUX_DIFF_0_3;
        case 6: return CONFIG_MUX_DIFF_1_3;
        case 7: return CONFIG_MUX_DIFF_2_3;
        default: return CONFIG_MUX_SINGLE_0;
    }
}

// Gain to PGA mapping
constexpr uint16_t getGainPGA(uint8_t gain) {
    switch (gain) {
        case 0: return CONFIG_PGA_6_144V;
        case 1: return CONFIG_PGA_4_096V;
        case 2: return CONFIG_PGA_2_048V;
        case 3: return CONFIG_PGA_1_024V;
        case 4: return CONFIG_PGA_0_512V;
        case 5: return CONFIG_PGA_0_256V;
        default: return CONFIG_PGA_2_048V;
    }
}

// Data rate to DR mapping
constexpr uint16_t getDataRateDR(uint8_t rate) {
    switch (rate) {
        case 0: return CONFIG_DR_8SPS;
        case 1: return CONFIG_DR_16SPS;
        case 2: return CONFIG_DR_32SPS;
        case 3: return CONFIG_DR_64SPS;
        case 4: return CONFIG_DR_128SPS;
        case 5: return CONFIG_DR_250SPS;
        case 6: return CONFIG_DR_475SPS;
        case 7: return CONFIG_DR_860SPS;
        default: return CONFIG_DR_128SPS;
    }
}

} // namespace ADS1115