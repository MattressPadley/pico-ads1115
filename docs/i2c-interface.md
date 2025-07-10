# ADS1115 I2C Interface

## I2C Protocol Support
The ADS1115 is fully compatible with I2C standard and supports:
- **Standard Mode**: 100 kHz
- **Fast Mode**: 400 kHz  
- **High-Speed Mode**: 3.4 MHz

## I2C Addressing

### Device Addresses
The ADS1115 supports four different I2C addresses based on the ADDR pin connection:

| ADDR Pin Connection | I2C Address (7-bit) | I2C Address (8-bit Write) | I2C Address (8-bit Read) |
|-------------------|-------------------|-------------------------|------------------------|
| GND              | 0x48              | 0x90                    | 0x91                   |
| VDD              | 0x49              | 0x92                    | 0x93                   |
| SDA              | 0x4A              | 0x94                    | 0x95                   |
| SCL              | 0x4B              | 0x96                    | 0x97                   |

### Multi-Device Configuration
- Up to 4 ADS1115 devices can be connected on a single I2C bus
- Each device must have a unique address (different ADDR pin connections)
- Total of 16 single-ended inputs possible with 4 devices

## Communication Protocol

### Write Operation (Register Configuration)
1. **Start Condition**
2. **Device Address + Write Bit** (7-bit address + 0)
3. **Pointer Register** (8-bit register address)
4. **Data Bytes** (16-bit data, MSB first)
5. **Stop Condition**

#### Example: Writing to Config Register
```
START | 0x90 | ACK | 0x01 | ACK | 0x85 | ACK | 0x83 | ACK | STOP
      |Device|     |Config|     | MSB  |     | LSB  |     |
      |Addr+W|     | Reg  |     |      |     |      |     |
```

### Read Operation (Data Retrieval)
1. **Start Condition**
2. **Device Address + Write Bit** (to set pointer)
3. **Pointer Register** (8-bit register address)
4. **Repeated Start Condition**
5. **Device Address + Read Bit**
6. **Data Bytes** (16-bit data, MSB first)
7. **Stop Condition**

#### Example: Reading Conversion Register
```
START | 0x90 | ACK | 0x00 | ACK | RESTART | 0x91 | ACK | MSB | ACK | LSB | NACK | STOP
      |Device|     |Conv  |     |         |Device|     |     |     |     |      |
      |Addr+W|     | Reg  |     |         |Addr+R|     |     |     |     |      |
```

## Timing Considerations

### Conversion Time
- **Conversion Duration**: Depends on data rate setting
- **Single-Shot Mode**: Wait for conversion completion before reading
- **Continuous Mode**: Read at any time (gets latest conversion)

### Data Rate vs Conversion Time
| Data Rate (SPS) | Conversion Time |
|----------------|-----------------|
| 8              | 125 ms          |
| 16             | 62.5 ms         |
| 32             | 31.25 ms        |
| 64             | 15.625 ms       |
| 128            | 7.8125 ms       |
| 250            | 4 ms            |
| 475            | 2.105 ms        |
| 860            | 1.163 ms        |

## Pull-up Resistors
- **SDA and SCL**: Require external pull-up resistors (typically 4.7kΩ to 10kΩ)
- **ALERT/RDY**: Requires external pull-up resistor if used
- **Pull-up voltage**: Should match the logic level of the I2C master

## Common I2C Commands

### Single-Shot Conversion Sequence
1. Write to Config register with desired settings and OS bit = 1
2. Wait for conversion time based on data rate
3. Read from Conversion register

### Continuous Conversion Setup
1. Write to Config register with MODE bit = 0 (continuous)
2. Read from Conversion register as needed

### Check Conversion Status
1. Read Config register
2. Check OS bit (bit 15): 1 = conversion complete, 0 = conversion in progress

## Error Handling
- **ACK/NACK**: Monitor acknowledgment bits for communication errors
- **Timeout**: Implement timeouts for I2C transactions
- **Bus Recovery**: Implement bus recovery procedures for stuck conditions