# ADS1115 Programming Guide

## Basic Operation Modes

### Single-Shot Mode
Single-shot mode performs one conversion and then powers down automatically.

**Advantages:**
- Lower power consumption
- Precise timing control
- Good for battery-powered applications

**Process:**
1. Configure the device settings
2. Start conversion by setting OS bit to 1
3. Wait for conversion completion
4. Read the result

### Continuous Mode  
Continuous mode performs conversions continuously at the programmed data rate.

**Advantages:**
- Always ready data available
- Good for monitoring applications
- Simpler programming model

**Process:**
1. Configure device with MODE bit = 0
2. Read conversion register as needed

## Configuration Examples

### Basic Single-Ended Reading (AIN0)
```
Config Register Value: 0xC383
- OS: 1 (Start conversion)
- MUX: 100 (AIN0 to GND)
- PGA: 001 (±4.096V range)
- MODE: 1 (Single-shot)
- DR: 100 (128 SPS)
- COMP_*: Default comparator settings
```

### Differential Reading (AIN0 - AIN1)
```
Config Register Value: 0xC083  
- OS: 1 (Start conversion)
- MUX: 000 (AIN0 - AIN1)
- PGA: 001 (±4.096V range)
- MODE: 1 (Single-shot)
- DR: 100 (128 SPS)
- COMP_*: Default comparator settings
```

### High-Speed Continuous Mode
```
Config Register Value: 0x01E3
- OS: 0 (No effect in continuous)
- MUX: 100 (AIN0 to GND)
- PGA: 001 (±4.096V range)  
- MODE: 0 (Continuous)
- DR: 111 (860 SPS)
- COMP_*: Default comparator settings
```

## Data Conversion

### Raw ADC to Voltage Conversion
```
Voltage = (ADC_Value × Full_Scale_Range) / 32767

Where Full_Scale_Range depends on PGA setting:
- PGA 2/3: ±6.144V
- PGA 1:   ±4.096V
- PGA 2:   ±2.048V
- PGA 4:   ±1.024V
- PGA 8:   ±0.512V
- PGA 16:  ±0.256V
```

### Example Calculations
For PGA = 1 (±4.096V range):
- ADC Value: 16383 → Voltage: +2.048V
- ADC Value: 0     → Voltage: 0V  
- ADC Value: -16384 → Voltage: -2.048V

## Programming Sequence

### Single-Shot Measurement
1. **Write Pointer Register**: 0x01 (Config register)
2. **Write Config Register**: Set desired configuration with OS=1
3. **Wait**: For conversion time based on data rate
4. **Write Pointer Register**: 0x00 (Conversion register)
5. **Read Conversion Register**: Get 16-bit result

### Continuous Measurement Setup
1. **Write Pointer Register**: 0x01 (Config register)
2. **Write Config Register**: Set MODE=0 for continuous
3. **Write Pointer Register**: 0x00 (Conversion register)
4. **Read Conversion Register**: Anytime for latest result

### Reading Multiple Channels
```
For each channel:
1. Configure MUX bits for desired channel
2. Start conversion (single-shot) or wait (continuous)
3. Read result
4. Repeat for next channel
```

## Alert/Comparator Programming

### Simple Threshold Alert
1. **Set Threshold Registers**:
   - Hi_thresh: Upper limit value
   - Lo_thresh: Lower limit value
2. **Configure Comparator**:
   - COMP_MODE: 0 (Traditional comparator)
   - COMP_POL: 0 (Active low alert)
   - COMP_QUE: Set assertion count
3. **Monitor ALERT/RDY Pin**: Goes low when threshold exceeded

### Window Comparator
1. **Set Threshold Registers**:
   - Hi_thresh: Upper window limit
   - Lo_thresh: Lower window limit  
2. **Configure Comparator**:
   - COMP_MODE: 1 (Window comparator)
   - Alert triggers when value is outside window

### Conversion Ready Detection
1. **Set Special Threshold Values**:
   - Hi_thresh: 0x8000 (MSB = 1)
   - Lo_thresh: 0x7FFF (MSB = 0)
2. **ALERT/RDY Pin**: Pulses low after each conversion

## Best Practices

### Power Management
- Use single-shot mode for battery applications
- Consider data rate vs power consumption trade-off
- Implement proper power sequencing

### Accuracy Considerations
- Allow settling time after PGA changes
- Use appropriate data rates for signal bandwidth
- Consider input impedance effects
- Implement proper grounding and PCB layout

### Error Handling
- Verify I2C communication with ACK checks
- Implement timeouts for conversion waits
- Validate configuration register writes
- Handle I2C bus errors gracefully

### Performance Optimization
- Use continuous mode for high-throughput applications
- Batch multiple channel reads efficiently
- Consider using ALERT/RDY for conversion timing
- Optimize I2C clock speed for application needs

## Common Pitfalls

1. **Not waiting for conversion completion** in single-shot mode
2. **Incorrect PGA setting** causing input range issues
3. **Missing pull-up resistors** on I2C lines
4. **Reading wrong register** due to pointer register confusion
5. **Ignoring conversion timing** requirements
6. **Improper threshold configuration** for comparator mode