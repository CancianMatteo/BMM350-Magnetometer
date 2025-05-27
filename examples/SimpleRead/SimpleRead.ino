#include "BMM350.h"

#define SDA_PIN 22
#define SCL_PIN 23

// Create an instance of the BMM350 class
BMM350 magnetometer(0x14); // or 0x15

void setup() {
    // Initialize serial communication for debugging
    Serial.begin(115200);
    while(!Serial);

    Serial.println("1/3 Initializing BMM350 Magnetometer...");
    // Initialize the magnetometer
    while (!magnetometer.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("Failed to initialize BMM350! Check your wiring.");
        delay(500);
    }
    Serial.println("2/3 BMM350 initialized");

    // Enable all axes
    magnetometer.setEnDisAbleAxisXYZ(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN);
    Serial.println("3/3 BMM350 ready");
}

// Variables to store magnetometer readings
float x, y, z;

void loop() {
    // Read magnetic field data
    if (magnetometer.readRawMagnetometerData(x, y, z)) {
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