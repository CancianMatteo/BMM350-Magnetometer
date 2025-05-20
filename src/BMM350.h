#pragma once

#include <Wire.h>
#include "utilities/BMM350_SensorAPI/bmm350.h"
#include "utilities/BMM350_SensorAPI/bmm350_defs.h"

// Default I2C address for BMM350
#define BMM350_I2C_ADDRESS 0x10

class BMM350 {
public:
    BMM350(uint8_t address = BMM350_I2C_ADDRESS);

    // Sensor initialization
    bool begin();

    // Magnetometer data reading
    bool readMagnetometer(float &x, float &y, float &z);

    // Calibration
    void setCalibration(float xOffset, float yOffset, float zOffset);
    void getCalibration(float &xOffset, float &yOffset, float &zOffset);

    // Power mode
    bool setPowerMode(uint8_t mode);
    uint8_t getPowerMode();

    bool setPresetMode(uint8_t preset);

    uint8_t getChipID();

    // Data rate
    bool setDataRate(uint8_t odr);
    uint8_t getDataRate();

    // Interrupts
    bool enableInterrupt(uint8_t int_type, bool enable);
    bool configureInterrupt(uint8_t int_type, uint8_t config);
    bool getInterruptStatus(uint8_t &status);

    // Thresholds
    bool setThreshold(uint8_t axis, float threshold);
    bool getThreshold(uint8_t axis, float &threshold);

private:
    // Low-level I2C communication
    int8_t i2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len);
    int8_t i2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len);

    // Delay function for API
    void delayMs(uint32_t ms);

    // Helper for register access
    void writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
};