#include <Wire.h>
#include "BMM350.h"

#define SDA_PIN 22
#define SCL_PIN 23

BMM350 magnetometer(0x14); // or 0x15

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing BMM350 Magnetometer...");
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  while (!magnetometer.begin(&Wire)) {
      Serial.println("Failed to initialize BMM350! Check your wiring.");
      delay(500);
  }
  Serial.println("BMM350 initialized and ready");

  magnetometer.setPresetMode(BMM350_PRESETMODE_HIGHACCURACY,BMM350_DATA_RATE_50HZ);
}

// Variables to store magnetometer readings
float x, y, z;

void loop() {
  magnetometer.readMagnetometerData(x, y, z);
    Serial.print("Raw:");
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(0);
    Serial.print(',');
    Serial.print(x*10);
    Serial.print(',');
    Serial.print(y*10);
    Serial.print(',');
    Serial.print(z*10);
    Serial.println();
  delay(100);
}
