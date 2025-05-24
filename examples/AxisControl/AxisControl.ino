#include "BMM350.h"

BMM350 magnetometer(0x14); // or 0x15

void setup() {
    Serial.begin(115200);

    // Initialize the magnetometer
    while (!magnetometer.begin(22, 23)) {
        Serial.println("Failed to initialize BMM350!");
        delay(500);
    }
    Serial.println("BMM350 initialized.");
    // Enable only X and Y axes
    magnetometer.setEnDisAbleAxisXYZ(BMM350_ENABLE_AXIS, BMM350_ENABLE_AXIS, BMM350_DISABLE_AXIS);
    Serial.println("BMM350 ready. Only X and Y axes enabled.");
}

void loop() {
    float x, y, z;
    bool axes[3];
    magnetometer.getAxisStateXYZ(axes);

    if (magnetometer.readMagnetometer(x, y, z)) {
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