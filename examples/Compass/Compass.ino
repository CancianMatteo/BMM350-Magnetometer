#include <Wire.h>
#include "BMM350.h"

BMM350 magnetometer(0x14); // or 0x15

void setup() {
    Serial.begin(115200);
    while(!Serial);

    Serial.println("Initializing BMM350 Magnetometer...");
    // Initialize the magnetometer
    Wire.begin();
    delay(100);
    while (!magnetometer.begin(&Wire)) {
        Serial.println("Failed to initialize BMM350!");
        delay(500);
    }
    Serial.println("BMM350 initialized and ready!");
}

void loop() {
    float heading = magnetometer.getHeadingDegree();
    Serial.print("Heading: ");
    Serial.print(heading, 2);
    Serial.println(" deg");
    delay(500);
}