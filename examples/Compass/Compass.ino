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
}

void loop() {
    float heading = magnetometer.getHeadingDegree();
    Serial.print("Heading: ");
    Serial.print(heading, 2);
    Serial.println(" deg");
    delay(500);
}