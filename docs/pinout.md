# ADS1115 Pinout and Pin Descriptions

## Pin Configuration (10-Pin Package)

| Pin | Name      | Type   | Description |
|-----|-----------|--------|-------------|
| 1   | ADDR      | Input  | I2C Address Select |
| 2   | ALERT/RDY | Output | Alert/Ready Signal |
| 3   | GND       | Power  | Ground |
| 4   | AIN0      | Input  | Analog Input Channel 0 |
| 5   | AIN1      | Input  | Analog Input Channel 1 |
| 6   | AIN2      | Input  | Analog Input Channel 2 |
| 7   | AIN3      | Input  | Analog Input Channel 3 |
| 8   | VDD       | Power  | Power Supply (2.0V to 5.5V) |
| 9   | SDA       | I/O    | I2C Serial Data |
| 10  | SCL       | Input  | I2C Serial Clock |

## Pin Descriptions

### Power Pins
- **VDD (Pin 8)**: Positive power supply pin, accepts 2.0V to 5.5V
- **GND (Pin 3)**: Ground reference for all signals

### Analog Input Pins
- **AIN0-AIN3 (Pins 4-7)**: Four analog input channels
  - Can be configured as 4 single-ended inputs
  - Can be configured as 2 differential pairs (AIN0-AIN1, AIN2-AIN3)
  - Input voltage range determined by PGA setting

### Digital Interface Pins
- **SDA (Pin 9)**: I2C Serial Data line (bidirectional)
- **SCL (Pin 10)**: I2C Serial Clock line (input)

### Control Pins
- **ADDR (Pin 1)**: I2C Address Select
  - Connect to GND: Address 0x48 (default)
  - Connect to VDD: Address 0x49
  - Connect to SDA: Address 0x4A
  - Connect to SCL: Address 0x4B

- **ALERT/RDY (Pin 2)**: Alert/Ready Output
  - Can function as conversion ready signal
  - Can function as comparator alert output
  - Active low output
  - Requires external pull-up resistor

## Module Breakout Board Pins
When using a breakout board module, pins are typically labeled as:
- **VCC/VDD**: Power supply input
- **GND**: Ground
- **SDA**: I2C data line
- **SCL**: I2C clock line
- **A0-A3**: Analog input channels
- **ADDR**: Address select pin
- **ALRT**: Alert/ready output pin