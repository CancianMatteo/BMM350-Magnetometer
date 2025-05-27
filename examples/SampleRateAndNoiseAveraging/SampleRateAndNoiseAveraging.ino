// Example: Set 100Hz sample rate and Ultra Low Noise mode on BMM350
#include "BMM350.h"

#define SDA_PIN 22
#define SCL_PIN 23

BMM350 magnetometer(0x14); // or 0x15

void setup() {
    Serial.begin(115200);
    while(!Serial);
    
    Serial.println("1/4 Initializing BMM350 Magnetometer...");
    // Initialize the magnetometer
    while (!magnetometer.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("Failed to initialize BMM350! Check your wiring.");
        delay(500);
    }
    Serial.println("2/4 BMM350 initialized");

    // Set 100Hz sample rate and Ultra Low Noise performance
    if (!magnetometer.setRateAndPerformance(BMM350_DATA_RATE_100HZ, BMM350_ULTRALOWNOISE)) {
        Serial.println("Failed to set rate/performance!");
    } else {
        Serial.println("3/4 Set to 100Hz, Ultra Low Noise mode.");
    }

    // Enable all axes
    magnetometer.setEnDisAbleAxisXYZ(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN);
    Serial.println("4/4 BMM350 ready");
}

void loop() {
    float x, y, z;
    if (magnetometer.readCompensatedGeomagneticData(x, y, z)) {
        Serial.print("X: "); Serial.print(x, 2);
        Serial.print("  Y: "); Serial.print(y, 2);
        Serial.print("  Z: "); Serial.println(z, 2);
    } else {
        Serial.println("Read error.");
    }
    delay(10); // ~100Hz loop
}