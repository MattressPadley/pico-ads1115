# ADS1115 Register Map and Configuration

## Register Overview
The ADS1115 has a simple register structure with four 16-bit registers accessible via I2C interface.

## Register Map

| Address | Register Name | Access | Description |
|---------|---------------|--------|-------------|
| 0x00    | Conversion    | R      | Contains the result of the last conversion |
| 0x01    | Config        | R/W    | Configuration settings for the ADC |
| 0x02    | Lo_thresh     | R/W    | Low threshold value for comparator |
| 0x03    | Hi_thresh     | R/W    | High threshold value for comparator |

## Pointer Register
- **Size**: 8 bits
- **Function**: Selects which register to read/write
- **Values**:
  - 0x00: Conversion register
  - 0x01: Config register  
  - 0x02: Lo_thresh register
  - 0x03: Hi_thresh register

## Conversion Register (0x00)
- **Size**: 16 bits
- **Access**: Read-only
- **Function**: Contains the 16-bit conversion result
- **Format**: Two's complement for differential measurements
- **Range**: -32768 to +32767 (signed 16-bit)

## Configuration Register (0x01)
- **Size**: 16 bits
- **Access**: Read/Write
- **Default**: 0x8583

### Bit Field Breakdown

| Bits  | Field | Description | Values |
|-------|-------|-------------|--------|
| 15    | OS    | Operational Status/Single-shot start | 0: No effect, 1: Start conversion |
| 14-12 | MUX   | Input multiplexer configuration | See MUX table below |
| 11-9  | PGA   | Programmable gain amplifier | See PGA table below |
| 8     | MODE  | Operating mode | 0: Continuous, 1: Single-shot |
| 7-5   | DR    | Data rate | See Data Rate table below |
| 4     | COMP_MODE | Comparator mode | 0: Traditional, 1: Window |
| 3     | COMP_POL | Comparator polarity | 0: Active low, 1: Active high |
| 2     | COMP_LAT | Comparator latching | 0: Non-latching, 1: Latching |
| 1-0   | COMP_QUE | Comparator queue | See Queue table below |

### MUX (Input Multiplexer) Configuration

| MUX[2:0] | Input Configuration |
|----------|-------------------|
| 000      | AIN0 - AIN1 (differential) |
| 001      | AIN0 - AIN3 (differential) |
| 010      | AIN1 - AIN3 (differential) |
| 011      | AIN2 - AIN3 (differential) |
| 100      | AIN0 - GND (single-ended) |
| 101      | AIN1 - GND (single-ended) |
| 110      | AIN2 - GND (single-ended) |
| 111      | AIN3 - GND (single-ended) |

### PGA (Programmable Gain Amplifier) Configuration

| PGA[2:0] | Gain | Full Scale Range |
|----------|------|------------------|
| 000      | 2/3  | ±6.144V |
| 001      | 1    | ±4.096V |
| 010      | 2    | ±2.048V |
| 011      | 4    | ±1.024V |
| 100      | 8    | ±0.512V |
| 101      | 16   | ±0.256V |
| 110      | 16   | ±0.256V |
| 111      | 16   | ±0.256V |

### Data Rate Configuration

| DR[2:0] | Samples per Second (SPS) |
|---------|-------------------------|
| 000     | 8 |
| 001     | 16 |
| 010     | 32 |
| 011     | 64 |
| 100     | 128 |
| 101     | 250 |
| 110     | 475 |
| 111     | 860 |

### Comparator Queue Configuration

| COMP_QUE[1:0] | Function |
|---------------|----------|
| 00            | Assert after 1 conversion |
| 01            | Assert after 2 conversions |
| 10            | Assert after 4 conversions |
| 11            | Disable comparator |

## Threshold Registers

### Lo_thresh Register (0x02)
- **Size**: 16 bits
- **Access**: Read/Write
- **Default**: 0x8000
- **Function**: Low threshold value for comparator

### Hi_thresh Register (0x03)
- **Size**: 16 bits
- **Access**: Read/Write  
- **Default**: 0x7FFF
- **Function**: High threshold value for comparator

## Special Configurations

### Conversion Ready Mode
To configure ALERT/RDY pin as conversion ready:
- Set Hi_thresh register MSB to 1 (0x8000)
- Set Lo_thresh register MSB to 0 (0x7FFF)

### Default Reset Values
- Config register: 0x8583
- Lo_thresh register: 0x8000
- Hi_thresh register: 0x7FFF