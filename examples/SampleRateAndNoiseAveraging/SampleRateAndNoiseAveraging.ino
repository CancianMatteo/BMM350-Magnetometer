// Example: Set 100Hz sample rate and Ultra Low Noise mode on BMM350
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
        Serial.println("Failed to initialize BMM350! Check your wiring.");
        delay(500);
    }
    Serial.println("2/3 BMM350 initialized and ready");

    // Set 100Hz sample rate and Ultra Low Noise performance
    if (magnetometer.setRateAndPerformance(BMM350_DATA_RATE_100HZ, BMM350_ULTRALOWNOISE)) {
        Serial.println("3/3 Set to 100Hz, Ultra Low Noise mode.");
    } else {
        Serial.println("Failed to set rate/performance!");
    }
}

void loop() {
    bmm350_mag_temp_data data = magnetometer.readCalibratedGeomagneticData();
    Serial.print("X: "); Serial.print(data.x, 2);
    Serial.print("  Y: "); Serial.print(data.y, 2);
    Serial.print("  Z: "); Serial.println(data.z, 2);
    delay(10); // ~100Hz loop
}