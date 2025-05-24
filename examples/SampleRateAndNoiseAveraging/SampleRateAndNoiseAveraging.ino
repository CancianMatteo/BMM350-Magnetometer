// Example: Set 100Hz sample rate and Ultra Low Noise mode on BMM350
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

    // Set 100Hz sample rate and Ultra Low Noise performance
    if (!magnetometer.setRateAndPerformance(BMM350_DATA_RATE_100HZ, BMM350_ULTRALOWNOISE)) {
        Serial.println("Failed to set rate/performance!");
    } else {
        Serial.println("Set to 100Hz, Ultra Low Noise mode.");
    }
}

void loop() {
    float x, y, z;
    if (magnetometer.readMagnetometer(x, y, z)) {
        Serial.print("X: "); Serial.print(x, 2);
        Serial.print("  Y: "); Serial.print(y, 2);
        Serial.print("  Z: "); Serial.println(z, 2);
    } else {
        Serial.println("Read error.");
    }
    delay(10); // ~100Hz loop
}