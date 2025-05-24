# Arduino IDE library for BMM350 Bosch's magnetometer

This library provides an easy-to-use interface for the Bosch BMM350 magnetometer sensor, enabling Arduino and compatible boards (such as ESP32) to read magnetic field data, configure sensor settings, and use advanced features like heading calculation and threshold interrupts.

---

## Features

- Simple API for reading magnetometer data (X, Y, Z axes)
- Heading (compass) calculation
- Calibration support
- Configurable sample rate and noise/performance modes
- Axis enable/disable
- Data ready and threshold interrupts
- Example sketches for quick start and advanced usage

---

## Installation

1. Download or clone this repository.
2. Copy the folder to your Arduino `libraries` directory (typically `Documents/Arduino/libraries`).
3. Restart the Arduino IDE.

---

## Usage

Include the library in your sketch:

```cpp
#include <BMM350.h>
```

Create an instance (specify the I2C address if needed):

```cpp
BMM350 magnetometer(0x14); // or 0x15, depending on your hardware
```

Initialize in `setup()` (for ESP32, specify SDA/SCL pins):

```cpp
void setup() {
    Serial.begin(115200);
    while (!magnetometer.begin(22, 23)) {
        Serial.println("Failed to initialize BMM350! Check your wiring and I2C address.");
        delay(500);
    }
    Serial.println("BMM350 initialized successfully.");
}
```

Read data in `loop()`:

```cpp
void loop() {
    float x, y, z;
    if (magnetometer.readMagnetometer(x, y, z)) {
        Serial.print("X: "); Serial.print(x, 2);
        Serial.print(" Y: "); Serial.print(y, 2);
        Serial.print(" Z: "); Serial.println(z, 2);
    } else {
        Serial.println("Failed to read magnetometer data.");
    }
    delay(500);
}
```

---

## Example Sketches

- **SimpleRead**: Basic reading of X, Y, Z magnetic field values.
- **Compass**: Calculate and print heading (compass angle).
- **SampleRateAndNoiseAveraging**: Set 100Hz sample rate and ultra low noise mode.
- **ThresholdInterrupt**: Use threshold interrupts to detect when a magnetic field exceeds a set value.

See the `examples/` folder for more.

---

## API Reference

- `bool begin(uint8_t SDA, uint8_t SCL)`: Initialize the sensor (specify SDA/SCL pins for ESP32/ESP8266).
- `bool readMagnetometer(float &x, float &y, float &z)`: Read magnetic field data.
- `void setCalibration(float xOffset, float yOffset, float zOffset)`: Set calibration offsets.
- `void getCalibration(float &xOffset, float &yOffset, float &zOffset)`: Get calibration offsets.
- `bool setPowerMode(bmm350_power_modes mode)`: Set power mode.
- `bmm350_power_modes getPowerMode()`: Get current power mode.
- `String getPowerModeString()`: Get power mode as a string.
- `uint8_t getChipID()`: Read sensor chip ID.
- `bool setRateAndPerformance(bmm350_data_rates rate, bmm350_performance_parameters performanceMode)`: Set sample rate and noise/performance.
- `float getSampleRate()`: Get current sample rate.
- `void setEnDisAbleAxisXYZ(...)`: Enable/disable axes.
- `void getAxisStateXYZ(bool enAxis[3])`: Get axis enable state.
- `bmm350_mag_temp_data getGeomagneticData()`: Get calibrated magnetic data.
- `float getHeadingDegree()`: Get compass heading in degrees.
- `bool enableInterrupt(bool enable)`: Enable/disable data ready interrupt.
- `bool configureInterrupt(...)`: Configure interrupt parameters.
- `bool getInterruptStatus(uint8_t &status)`: Get interrupt status.
- `bool setDataReadyPin(...)`: Configure data ready pin.
- `bool getDataReadyState()`: Check if new data is ready.
- `void setThresholdInterrupt(...)`: Configure threshold interrupt.
- `sBmm350ThresholdData_t getThresholdData()`: Get threshold interrupt data.
- `void softReset()`: Soft reset the sensor.

---

## Troubleshooting

- **Sensor not detected:**  
  - Run an I2C scanner to confirm the address (should be 0x14 or 0x15).
  - Check wiring (SDA/SCL, GND, 3.3V) and ensure pull-up resistors are present.
  - Use the correct I2C address in your sketch.

- **Strange or stuck readings:**  
  - Ensure only one instance of the library is using the I2C bus.
  - Power cycle the sensor and board.
  - Check for duplicate or static global variables in your code.

- **Initialization fails:**  
  - Double-check wiring and power supply.
  - Confirm the sensor is not 5V tolerant—use 3.3V only.

---

## License

This library is released under the GNU General Public License v3.0 (GPL-3.0).  
See the LICENSE file for more details.

---