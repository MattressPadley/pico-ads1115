# ADS1115 Technical Specifications

## Electrical Characteristics

### Power Supply
- **Supply Voltage (VDD)**: 2.0V to 5.5V
- **Current Consumption**:
  - Continuous Mode: 150µA (typical)
  - Single-Shot Mode: Auto shut-down after conversion
- **Power-On Reset**: Automatic on power-up

### Performance Specifications
- **Resolution**: 16 bits
- **Sample Rate**: 8, 16, 32, 64, 128, 250, 475, 860 samples per second (SPS)
- **Input Voltage Range**: Determined by PGA setting, up to ±6.144V
- **PGA Voltage Ranges**: ±256mV, ±512mV, ±1.024V, ±2.048V, ±4.096V, ±6.144V
- **Programmable Gain**: 2/3, 1, 2, 4, 8, 16

### Input Configuration
- **Input Channels**: 4 single-ended or 2 differential
- **Input Multiplexer**: Selectable input channel configuration
- **Input Impedance**: High impedance inputs

### Digital Interface
- **Protocol**: I2C-compatible
- **Clock Speed**: Standard mode, fast mode, and high-speed mode compatible
- **I2C Addresses**: 0x48 (default), 0x49, 0x4A, 0x4B (selectable via ADDR pin)

## Environmental Specifications
- **Operating Temperature**: -40°C to +125°C
- **Storage Temperature**: -65°C to +150°C

## Package Information
- **Package Types**: QFN-10 (leadless), MSOP-10
- **Pin Count**: 10 pins
- **Package Size**: Ultra-small form factor