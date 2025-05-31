#include <Wire.h>
#include "BMM350.h"

BMM350 magnetometer(0x14); // or 0x15

void setup() {
    Serial.begin(115200);
    while(!Serial);

    Serial.println("Initializing BMM350 Magnetometer...");
    // Initialize the magnetometer
    Wire.begin();
    delay(100);
    while (!magnetometer.begin(&Wire)) {
        Serial.println("Failed to initialize BMM350! Check your wiring.");
        delay(500);
    }
    Serial.println("BMM350 initialized and ready");
}

// Variables to store magnetometer readings
float x, y, z;

void loop() {
    // Read magnetic field data
    if (magnetometer.readMagnetometerData(x, y, z)) {
        Serial.print("Magnetic Field: X=");
        Serial.print(x, 2);
        Serial.print(", Y=");
        Serial.print(y, 2);
        Serial.print(", Z=");
        Serial.println(z, 2);
    } else {
        Serial.println("Failed to read magnetometer data.");
    }

    // Wait for a short period before the next reading
    delay(500);
}