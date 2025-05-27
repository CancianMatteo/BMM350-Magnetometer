#include "BMM350.h"

#define SDA_PIN 22
#define SCL_PIN 23

BMM350 magnetometer(0x14); // or 0x15

void setup() {
    Serial.begin(115200);
    while(!Serial);

    Serial.println("1/3 Initializing BMM350 Magnetometer...");
    // Initialize the magnetometer
    while (!magnetometer.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("Failed to initialize BMM350!");
        delay(500);
    }
    Serial.println("2/3 BMM350 initialized");

    // Enable all axes
    magnetometer.setEnDisAbleAxisXYZ(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN);
    Serial.println("3/3 BMM350 ready");
}

void loop() {
    float heading = magnetometer.getHeadingDegree();
    Serial.print("Heading: ");
    Serial.print(heading, 2);
    Serial.println(" deg");
    delay(500);
}