#include <Wire.h>
#include "BMM350.h"

BMM350 magnetometer(0x14); // or 0x15
#define BMI160_I2C_ADDRESS 0x69

#define BMI160_CMD 0x7E
#define BMI160_ACC_CONF 0x40
#define BMI160_ACC_RANGE 0x41
#define BMI160_GYR_CONF 0x42
#define BMI160_GYR_RANGE 0x43

float aRes, gRes;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  delay(100);

  while (!magnetometer.begin(&Wire)) {
      Serial.println("Failed to initialize BMM350! Check your wiring.");
      delay(500);
  }

  magnetometer.setRateAndPerformance(BMM350_DATA_RATE_50HZ, BMM350_ULTRALOWNOISE);

  // Try to initialize BMI160
  writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_CMD, 0xB6);	    // Toggle softreset (and suspend to reset the BMI160)
	delay(100);
  /** BMI160 Accel & Gyro Power mode:
   * 0x10 - Suspend mode for accelerometer
   * 0x11 - Normal mode for accelerometer
   * 0x12 - Low power mode for accelerometer (lower ODR)
   * 0x14 - Suspend mode for gyroscope
   * 0x15 - Normal mode for gyroscope
   */
	writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_CMD, 0x11);	    // Start up again accelerometer
	delay(100);
	writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_CMD, 0x15);		// Start up again gyroscope
	delay(100);
	writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_ACC_RANGE, 0x0C);  // Set accel range: 0x0C=±16G, 0x08=±8G, 0x05=±4G, 0x03=±2G
	writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_GYR_RANGE, 0x01);  // Set gyro range: 0x00=±2000dps, 0x01=±1000dps, 0x02=±500dps, 0x03=±250dps
  /** Set accelerometer and gyroscope ODR and Bandwidth Parameter / Filter Mode
   *  
   * Format: 0xXY[7:4] = ODR, 0xXY[3:2] = BWP, 0xXY[1:0] = Filter perf mode (only for accelerometer), Reserved for gyro
   *  
   * Common values (ODR - bits [7:4]):
   *  - 0x01: ODR = 0.78125Hz; BWP = OSR1
   *  - 0x02: ODR = 1.5625Hz; BWP = OSR1
   *  - 0x03: ODR = 3.125Hz; BWP = OSR1
   *  - 0x04: ODR = 6.25Hz; BWP = OSR1
   *  - 0x05: ODR = 12.5Hz; BWP = OSR1
   *  - 0x06: ODR = 25Hz; BWP = OSR1
   *  - 0x07: ODR = 50Hz; BWP = OSR1
   *  - 0x27: ODR = 50Hz; BWP = OSR4
   *  - 0x08: ODR = 100Hz; BWP = OSR1
   *  - 0x18: ODR = 100Hz; BWP = OSR2
   *  - 0x28: ODR = 100Hz; BWP = OSR4
   *  - 0x29: ODR = 200Hz; BWP = OSR4
   *  - 0x2A: ODR = 400Hz; BWP = OSR4
   * 
   * Bandwidth Parameter (BWP - bits [3:2]):
   *  00 → OSR1 (no averaging, lowest latency)
   *  01 → OSR2 (oversample by 2)
   *  10 → OSR4 (oversample by 4)
   * 
   * Filter Performance Mode (bits [1:0] for accelerometer):
   *  00 → Normal (default)
   *  01 → Low power (lower ODR, higher noise)
   *  10 → High performance (higher ODR, lower noise)
   */
	writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_ACC_CONF, 0x27);  
	writeByteI2C(Wire, BMI160_I2C_ADDRESS, BMI160_GYR_CONF, 0x27);

  // Perform accelerometer auto-calibration
  autoCalibrateAccelerometer();

  aRes = 16.f / 32768.f;			// ares value for full range (16g) readings
	gRes = 1000.f / 32768.f;	    // gres value for full range (1000dps) readings
}

void loop() {
  // Variables to store readings
  float ax=0, ay=0, az=0, gx=0, gy=0, gz=0, mx=0, my=0, mz=0;

  readAccelGyroData(ax, ay, az, gx, gy, gz);

  magnetometer.readMagnetometerData(mx, my, mz);

  Serial.print("Raw:");
  Serial.print(ax);
  Serial.print(',');
  Serial.print(ay);
  Serial.print(',');
  Serial.print(az);
  Serial.print(',');
  Serial.print(gx);
  Serial.print(',');
  Serial.print(gy);
  Serial.print(',');
  Serial.print(gz);
  Serial.print(',');
  Serial.print(mx);
  Serial.print(',');
  Serial.print(my);
  Serial.print(',');
  Serial.print(mz);
  Serial.println();

  delay(40);
}

void autoCalibrateAccelerometer() {
  // Configure accelerometer for auto-calibration
  Wire.beginTransmission(BMI160_I2C_ADDRESS);
  Wire.write(0x7E); // Command register
  Wire.write(0x37); // Start accelerometer offset calibration
  Wire.endTransmission();
  // Wait for calibration to complete
  delay(1000);
}

void readAccelGyroData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[12];                                          // x/y/z accel register data stored here

	readBytesI2C(Wire, BMI160_I2C_ADDRESS, 0x0C, 12, &rawData[0]);    // Read the 12 raw data registers into data array

	IMUCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];		  // Turn the MSB and LSB into a signed 16-bit value
	IMUCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	IMUCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	IMUCount[3] = ((int16_t)rawData[7] << 8) | rawData[6];
	IMUCount[4] = ((int16_t)rawData[9] << 8) | rawData[8];
	IMUCount[5] = ((int16_t)rawData[11] << 8) | rawData[10];

	// Calculate the accel value into actual g's per second
	ax = (float)IMUCount[3] * aRes;
	ay = (float)IMUCount[4] * aRes;
	az = (float)IMUCount[5] * aRes;

	// Calculate the gyro value into actual degrees per second
	gx = (float)IMUCount[0] * gRes;
	gy = (float)IMUCount[1] * gRes;
	gz = (float)IMUCount[2] * gRes;
}

void writeByteI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t data) {
  wire.beginTransmission(address);  // Initialize the Tx buffer
  wire.write(subAddress);           // Put slave register address in Tx buffer
  wire.write(data);                 // Put data in Tx buffer
  wire.endTransmission();           // Send the Tx buffer
}

void readBytesI2C(TwoWire& wire, uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
  wire.beginTransmission(address);  // Initialize the Tx buffer
  wire.write(subAddress);           // Put slave register address in Tx buffer
  wire.endTransmission(false);      // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  wire.requestFrom(address, count); // Read bytes from slave register address
  while (wire.available()) {
      dest[i++] = wire.read();      // Put read results in the Rx buffer
  }
}