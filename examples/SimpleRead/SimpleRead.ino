#include "BMM350.h"

// Create an instance of the BMM350 class
BMM350 magnetometer;

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);
    delay(3000);

    Serial.println("Initializing BMM350 Magnetometer...");

    // Initialize the magnetometer
    if (!magnetometer.begin(22, 23)) {
        Serial.println("Failed to initialize BMM350! Check your wiring.");
        delay(500);
    }

    Serial.println("BMM350 initialized successfully.");
}

// Variables to store magnetometer readings
float x, y, z;

void loop() {
    // Read magnetic field data
    if (magnetometer.readMagnetometer(x, y, z)) {
        Serial.print("Magnetic Field [uT]: X=");
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