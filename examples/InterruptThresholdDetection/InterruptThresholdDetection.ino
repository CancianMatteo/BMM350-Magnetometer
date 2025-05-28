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
    Serial.println("2/3 BMM350 initialized and ready");
    
    // Set threshold interrupt for X axis, high polarity
    magnetometer.setThresholdInterrupt(HIGH_THRESHOLD_INTERRUPT, 40, BMM350_ACTIVE_HIGH);
    Serial.println("3/3 Threshold interrupt set");
}

void loop() {
    // Check if new data is ready
    if (magnetometer.getDataReadyState()) {
        sBmm350ThresholdData_t thresholdData = magnetometer.getThresholdData();
        Serial.print("Thresholds: ");
        if (thresholdData.interrupt_x) {
            Serial.print("X exceeded: "); Serial.print(thresholdData.mag_x, 2); Serial.print(" ");
        }
        if (thresholdData.interrupt_y) {
            Serial.print("Y exceeded: "); Serial.print(thresholdData.mag_y, 2); Serial.print(" ");
        }
        if (thresholdData.interrupt_z) {
            Serial.print("Z exceeded: "); Serial.print(thresholdData.mag_z, 2); Serial.print(" ");
        }
        if (!thresholdData.interrupt_x && !thresholdData.interrupt_y && !thresholdData.interrupt_z) {
            Serial.print("No axis exceeded threshold.");
        }
        Serial.println();
    }
    delay(100);
}