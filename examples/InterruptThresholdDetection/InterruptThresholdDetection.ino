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
    
    // Set threshold interrupt for X axis, high polarity
    magnetometer.setThresholdInterrupt(HIGH_THRESHOLD_INTERRUPT, 40, BMM350_ACTIVE_HIGH);
    Serial.println("BMM350 ready. Threshold interrupt set.");
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