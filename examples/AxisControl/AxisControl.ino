#include <Wire.h>
#include "BMM350.h"

#define SDA_PIN 22
#define SCL_PIN 23

BMM350 magnetometer(0x14); // or 0x15

void setup() {
    Serial.begin(115200);
    while(!Serial);

    Serial.println("1/3 Initializing BMM350 Magnetometer...");
    // Initialize the magnetometer
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    while (!magnetometer.begin(&Wire)) {
        Serial.println("Failed to initialize BMM350!");
        delay(500);
    }
    Serial.println("2/3 BMM350 initialized");

    // Enable only X and Y axes
    magnetometer.setEnDisAbleAxesXYZ(BMM350_ENABLE_AXIS, BMM350_ENABLE_AXIS, BMM350_DISABLE_AXIS);
    Serial.println("3/3 BMM350 ready. Only X and Y axes enabled.");
}

void loop() {
    float x, y, z;
    bool axes[3];
    magnetometer.getAxesStateXYZ(axes);

    if (magnetometer.readMagnetometerData(x, y, z)) {
        Serial.print("Enabled Axes: ");
        if (axes[0]) Serial.print("X ");
        if (axes[1]) Serial.print("Y ");
        if (axes[2]) Serial.print("Z ");
        Serial.print(" | Data: ");
        if (axes[0]) { Serial.print("X="); Serial.print(x, 2); Serial.print(" "); }
        if (axes[1]) { Serial.print("Y="); Serial.print(y, 2); Serial.print(" "); }
        if (axes[2]) { Serial.print("Z="); Serial.print(z, 2); }
        Serial.println();
    } else {
        Serial.println("Read error.");
    }
    delay(500);
}