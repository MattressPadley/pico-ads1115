/**
 * @file ads1115_registers.hpp
 * @brief Register definitions and constants for the ADS1115 ADC
 *
 * This header contains all register addresses, bit field definitions,
 * configuration constants, and timing parameters for the ADS1115 device.
 * It provides low-level hardware interface definitions used by the main
 * library implementation.
 *
 */

#pragma once

#include <cstdint>

/**
 * @namespace ADS1115
 * @brief Contains all classes, functions, and types for the ADS1115 ADC library
 */
namespace ADS1115 {

/**
 * @name I2C Addresses
 * @brief Available I2C addresses for ADS1115 devices
 * @{
 */

/** @brief Default I2C address when ADDR pin is connected to GND */
constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x48;

/** @brief I2C address when ADDR pin is connected to GND */
constexpr uint8_t I2C_ADDRESS_0x48 = 0x48;
/** @brief I2C address when ADDR pin is connected to VDD */
constexpr uint8_t I2C_ADDRESS_0x49 = 0x49;
/** @brief I2C address when ADDR pin is connected to SDA */
constexpr uint8_t I2C_ADDRESS_0x4A = 0x4A;
/** @brief I2C address when ADDR pin is connected to SCL */
constexpr uint8_t I2C_ADDRESS_0x4B = 0x4B;

/** @} */

/**
 * @name Register Addresses
 * @brief ADS1115 internal register addresses
 * @{
 */

/** @brief Conversion result register (16-bit, read-only) */
constexpr uint8_t REG_CONVERSION = 0x00;
/** @brief Configuration register (16-bit, read-write) */
constexpr uint8_t REG_CONFIG = 0x01;
/** @brief Low threshold register (16-bit, read-write) */
constexpr uint8_t REG_LO_THRESH = 0x02;
/** @brief High threshold register (16-bit, read-write) */
constexpr uint8_t REG_HI_THRESH = 0x03;

/** @} */

/**
 * @name Configuration Register Bit Definitions
 * @brief Bit field definitions for the 16-bit configuration register
 * @{
 */

/**
 * @name Operational Status Bits (Bit 15)
 * @brief Control and status for conversion operations
 * @{
 */
/** @brief Operational status bit mask */
constexpr uint16_t CONFIG_OS_MASK = 0x8000;
/** @brief Start single conversion (write) */
constexpr uint16_t CONFIG_OS_SINGLE = 0x8000;
/** @brief Device is performing conversion (read) */
constexpr uint16_t CONFIG_OS_BUSY = 0x0000;
/** @brief Device is not performing conversion (read) */
constexpr uint16_t CONFIG_OS_NOT_BUSY = 0x8000;
/** @} */

/**
 * @name Input Multiplexer Configuration (Bits 14:12)
 * @brief Input channel selection bit definitions
 * @{
 */
/** @brief Input multiplexer bit mask */
constexpr uint16_t CONFIG_MUX_MASK = 0x7000;
/** @brief Differential: P=AIN0, N=AIN1 */
constexpr uint16_t CONFIG_MUX_DIFF_0_1 = 0x0000;
/** @brief Differential: P=AIN0, N=AIN3 */
constexpr uint16_t CONFIG_MUX_DIFF_0_3 = 0x1000;
/** @brief Differential: P=AIN1, N=AIN3 */
constexpr uint16_t CONFIG_MUX_DIFF_1_3 = 0x2000;
/** @brief Differential: P=AIN2, N=AIN3 */
constexpr uint16_t CONFIG_MUX_DIFF_2_3 = 0x3000;
/** @brief Single-ended: AIN0 to GND */
constexpr uint16_t CONFIG_MUX_SINGLE_0 = 0x4000;
/** @brief Single-ended: AIN1 to GND */
constexpr uint16_t CONFIG_MUX_SINGLE_1 = 0x5000;
/** @brief Single-ended: AIN2 to GND */
constexpr uint16_t CONFIG_MUX_SINGLE_2 = 0x6000;
/** @brief Single-ended: AIN3 to GND */
constexpr uint16_t CONFIG_MUX_SINGLE_3 = 0x7000;
/** @} */

/**
 * @name Programmable Gain Amplifier Configuration (Bits 11:9)
 * @brief Gain and voltage range selection bit definitions
 * @{
 */
/** @brief PGA configuration bit mask */
constexpr uint16_t CONFIG_PGA_MASK = 0x0E00;
/** @brief PGA: ±6.144V range (gain = 2/3) */
constexpr uint16_t CONFIG_PGA_6_144V = 0x0000;
/** @brief PGA: ±4.096V range (gain = 1) */
constexpr uint16_t CONFIG_PGA_4_096V = 0x0200;
/** @brief PGA: ±2.048V range (gain = 2, default) */
constexpr uint16_t CONFIG_PGA_2_048V = 0x0400;
/** @brief PGA: ±1.024V range (gain = 4) */
constexpr uint16_t CONFIG_PGA_1_024V = 0x0600;
/** @brief PGA: ±0.512V range (gain = 8) */
constexpr uint16_t CONFIG_PGA_0_512V = 0x0800;
/** @brief PGA: ±0.256V range (gain = 16) */
constexpr uint16_t CONFIG_PGA_0_256V = 0x0A00;
/** @} */

/**
 * @name Device Operating Mode (Bit 8)
 * @brief Operating mode selection bit definitions
 * @{
 */
/** @brief Operating mode bit mask */
constexpr uint16_t CONFIG_MODE_MASK = 0x0100;
/** @brief Continuous conversion mode */
constexpr uint16_t CONFIG_MODE_CONTIN = 0x0000;
/** @brief Single-shot conversion mode (default) */
constexpr uint16_t CONFIG_MODE_SINGLE = 0x0100;
/** @} */

/**
 * @name Data Rate Configuration (Bits 7:5)
 * @brief Conversion rate selection bit definitions
 * @{
 */
/** @brief Data rate bit mask */
constexpr uint16_t CONFIG_DR_MASK = 0x00E0;
/** @brief 8 samples per second */
constexpr uint16_t CONFIG_DR_8SPS = 0x0000;
/** @brief 16 samples per second */
constexpr uint16_t CONFIG_DR_16SPS = 0x0020;
/** @brief 32 samples per second */
constexpr uint16_t CONFIG_DR_32SPS = 0x0040;
/** @brief 64 samples per second */
constexpr uint16_t CONFIG_DR_64SPS = 0x0060;
/** @brief 128 samples per second (default) */
constexpr uint16_t CONFIG_DR_128SPS = 0x0080;
/** @brief 250 samples per second */
constexpr uint16_t CONFIG_DR_250SPS = 0x00A0;
/** @brief 475 samples per second */
constexpr uint16_t CONFIG_DR_475SPS = 0x00C0;
/** @brief 860 samples per second */
constexpr uint16_t CONFIG_DR_860SPS = 0x00E0;
/** @} */

/**
 * @name Comparator Mode (Bit 4)
 * @brief Comparator operation mode bit definitions
 * @{
 */
/** @brief Comparator mode bit mask */
constexpr uint16_t CONFIG_CMODE_MASK = 0x0010;
/** @brief Traditional comparator mode (default) */
constexpr uint16_t CONFIG_CMODE_TRAD = 0x0000;
/** @brief Window comparator mode */
constexpr uint16_t CONFIG_CMODE_WINDOW = 0x0010;
/** @} */

/**
 * @name Comparator Polarity (Bit 3)
 * @brief Alert pin polarity bit definitions
 * @{
 */
/** @brief Comparator polarity bit mask */
constexpr uint16_t CONFIG_CPOL_MASK = 0x0008;
/** @brief Alert pin active low (default) */
constexpr uint16_t CONFIG_CPOL_ACTVLOW = 0x0000;
/** @brief Alert pin active high */
constexpr uint16_t CONFIG_CPOL_ACTVHI = 0x0008;
/** @} */

/**
 * @name Comparator Latching (Bit 2)
 * @brief Alert latching behavior bit definitions
 * @{
 */
/** @brief Comparator latching bit mask */
constexpr uint16_t CONFIG_CLAT_MASK = 0x0004;
/** @brief Non-latching alert (default) */
constexpr uint16_t CONFIG_CLAT_NONLAT = 0x0000;
/** @brief Latching alert (requires manual clear) */
constexpr uint16_t CONFIG_CLAT_LATCH = 0x0004;
/** @} */

/**
 * @name Comparator Queue (Bits 1:0)
 * @brief Comparator alert queue bit definitions
 * @{
 */
/** @brief Comparator queue bit mask */
constexpr uint16_t CONFIG_CQUE_MASK = 0x0003;
/** @brief Assert alert after one conversion */
constexpr uint16_t CONFIG_CQUE_1CONV = 0x0000;
/** @brief Assert alert after two consecutive conversions */
constexpr uint16_t CONFIG_CQUE_2CONV = 0x0001;
/** @brief Assert alert after four consecutive conversions */
constexpr uint16_t CONFIG_CQUE_4CONV = 0x0002;
/** @brief Disable comparator (default) */
constexpr uint16_t CONFIG_CQUE_NONE = 0x0003;
/** @} */

/** @} */

/**
 * @name Default Configuration Values
 * @brief Predefined configuration constants
 * @{
 */

/**
 * @brief Default configuration register value
 *
 * Combines default settings for all configuration fields:
 * - Single conversion start
 * - Single-ended AIN0 input
 * - ±2.048V range (gain = 2)
 * - Single-shot mode
 * - 128 SPS data rate
 * - Traditional comparator
 * - Active low alert polarity
 * - Non-latching alert
 * - Comparator disabled
 */
constexpr uint16_t DEFAULT_CONFIG =
    CONFIG_OS_SINGLE |    // Start single conversion
    CONFIG_MUX_SINGLE_0 | // Single-ended AIN0
    CONFIG_PGA_2_048V |   // ±2.048V range
    CONFIG_MODE_SINGLE |  // Single-shot mode
    CONFIG_DR_128SPS |    // 128 samples per second
    CONFIG_CMODE_TRAD |   // Traditional comparator
    CONFIG_CPOL_ACTVLOW | // Active low
    CONFIG_CLAT_NONLAT |  // Non-latching
    CONFIG_CQUE_NONE;     // Disable comparator

/**
 * @brief Default high threshold value (maximum positive)
 */
constexpr int16_t DEFAULT_HIGH_THRESHOLD = 0x7FFF;

/**
 * @brief Default low threshold value (maximum negative)
 */
constexpr int16_t DEFAULT_LOW_THRESHOLD = 0x8000;

/** @} */

/**
 * @name Timing Constants
 * @brief Device timing and delay constants
 * @{
 */

/** @brief Power-on stabilization delay in milliseconds */
constexpr uint16_t POWER_ON_DELAY_MS = 25;
/** @brief Reset stabilization delay in milliseconds */
constexpr uint16_t RESET_DELAY_MS = 10;
/** @brief I2C communication timeout in milliseconds */
constexpr uint8_t I2C_TIMEOUT_MS = 100;

/** @} */

/**
 * @name ADC Value Limits and Resolution
 * @brief Constants defining ADC value ranges and resolution
 * @{
 */

/** @brief Maximum positive ADC value (15-bit signed) */
constexpr int16_t ADC_MAX_VALUE = 32767;
/** @brief Minimum negative ADC value (15-bit signed) */
constexpr int16_t ADC_MIN_VALUE = -32768;
/** @brief ADC resolution in bits */
constexpr uint16_t ADC_RESOLUTION = 16;
/** @brief Maximum unsigned 16-bit value */
constexpr uint16_t ADC_MAX_UNSIGNED = 65535;

/** @} */

/**
 * @name Conversion Result Register Constants
 * @brief Constants for interpreting conversion results
 * @{
 */

/** @brief Number of bits in conversion result */
constexpr uint8_t CONVERSION_RESULT_BITS = 16;
/** @brief Bit shift for conversion result (no shift for 16-bit) */
constexpr uint8_t CONVERSION_RESULT_SHIFT = 0;

/** @} */

/**
 * @name Reset Values
 * @brief Default register values after device reset
 * @{
 */

/** @brief Configuration register value after device reset */
constexpr uint16_t CONFIG_RESET_VALUE = 0x8583;

/** @} */

/**
 * @name Bit Manipulation Macros
 * @brief Utility macros for register bit manipulation
 * @{
 */

/** @brief Set specific bits in a register */
#define SET_BITS(reg, mask, value) ((reg & ~mask) | (value & mask))
/** @brief Get specific bits from a register */
#define GET_BITS(reg, mask) (reg & mask)
/** @brief Clear specific bits in a register */
#define CLEAR_BITS(reg, mask) (reg & ~mask)

/** @} */

/**
 * @name Register Value Mapping Functions
 * @brief Functions to convert enum values to register bit patterns
 * @{
 */

/**
 * @brief Convert channel index to MUX configuration bits
 *
 * @param channel Channel index (0-7)
 * @return MUX configuration bits for the specified channel
 */
constexpr uint16_t getChannelMux(uint8_t channel) {
  switch (channel) {
  case 0:
    return CONFIG_MUX_SINGLE_0; // A0
  case 1:
    return CONFIG_MUX_SINGLE_1; // A1
  case 2:
    return CONFIG_MUX_SINGLE_2; // A2
  case 3:
    return CONFIG_MUX_SINGLE_3; // A3
  case 4:
    return CONFIG_MUX_DIFF_0_1; // A0-A1
  case 5:
    return CONFIG_MUX_DIFF_0_3; // A0-A3
  case 6:
    return CONFIG_MUX_DIFF_1_3; // A1-A3
  case 7:
    return CONFIG_MUX_DIFF_2_3; // A2-A3
  default:
    return CONFIG_MUX_SINGLE_0; // Default to A0
  }
}

/**
 * @brief Convert gain index to PGA configuration bits
 *
 * @param gain Gain index (0-5)
 * @return PGA configuration bits for the specified gain
 */
constexpr uint16_t getGainPGA(uint8_t gain) {
  switch (gain) {
  case 0:
    return CONFIG_PGA_6_144V; // 2/3x gain
  case 1:
    return CONFIG_PGA_4_096V; // 1x gain
  case 2:
    return CONFIG_PGA_2_048V; // 2x gain (default)
  case 3:
    return CONFIG_PGA_1_024V; // 4x gain
  case 4:
    return CONFIG_PGA_0_512V; // 8x gain
  case 5:
    return CONFIG_PGA_0_256V; // 16x gain
  default:
    return CONFIG_PGA_2_048V; // Default to 2x
  }
}

/**
 * @brief Convert data rate index to DR configuration bits
 *
 * @param rate Data rate index (0-7)
 * @return Data rate configuration bits for the specified rate
 */
constexpr uint16_t getDataRateDR(uint8_t rate) {
  switch (rate) {
  case 0:
    return CONFIG_DR_8SPS; // 8 SPS
  case 1:
    return CONFIG_DR_16SPS; // 16 SPS
  case 2:
    return CONFIG_DR_32SPS; // 32 SPS
  case 3:
    return CONFIG_DR_64SPS; // 64 SPS
  case 4:
    return CONFIG_DR_128SPS; // 128 SPS (default)
  case 5:
    return CONFIG_DR_250SPS; // 250 SPS
  case 6:
    return CONFIG_DR_475SPS; // 475 SPS
  case 7:
    return CONFIG_DR_860SPS; // 860 SPS
  default:
    return CONFIG_DR_128SPS; // Default to 128 SPS
  }
}

/** @} */

} // namespace ADS1115
