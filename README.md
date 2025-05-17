## Arduino IDE library for BMM350 Bosch's new magnetometer sensor

This library provides an interface for the BMM350 magnetometer sensor from Bosch. It allows users to easily integrate the sensor into their Arduino projects, enabling the reading of magnetic field data and configuration of sensor settings.

### Features
- Easy-to-use API for reading magnetometer data
- Configuration options for sensor settings
- Example sketch for quick start

### Installation
1. Download the library from the repository.
2. Extract the contents to your Arduino libraries folder (usually located in `Documents/Arduino/libraries`).
3. Restart the Arduino IDE.

### Usage
To use the BMM350 library in your project, include the header file:

```cpp
#include <BMM350.h>
```

### Example
Refer to the `examples/SimpleRead/SimpleRead.ino` for a basic example of how to use the library to read magnetometer data.

### API Reference
- `BMM350::begin()`: Initializes the sensor.
- `BMM350::readData()`: Reads the current magnetometer data.
- `BMM350::setCalibration()`: Configures calibration settings for the sensor.

### License
This library is released under the MIT License. See the LICENSE file for more details.