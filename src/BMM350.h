#pragma once

#include <Wire.h>
#include "utilities/BMM350_SensorAPI/bmm350.h"
#include "utilities/BMM350_SensorAPI/bmm350_defs.h"

// Default I2C address for BMM350
#define BMM350_I2C_ADDRESS 0x14 // or 0x15

// Forward declarations for custom types if not included in bmm350_defs.h
#ifndef BMM350_MAG_TEMP_DATA_DEFINED
#define BMM350_MAG_TEMP_DATA_DEFINED

#define LOW_THRESHOLD_INTERRUPT  0
#define HIGH_THRESHOLD_INTERRUPT 1
#define NO_DATA                  0x7FFFFFFF

typedef struct {
    float mag_x;
    float mag_y;
    float mag_z;
    uint8_t interrupt_x;
    uint8_t interrupt_y;
    uint8_t interrupt_z;
} sBmm350ThresholdData_t;

#endif

class BMM350 {
public:
    BMM350(uint8_t address = BMM350_I2C_ADDRESS);

    // Sensor initialization
    bool begin(uint8_t SDA, uint8_t SCL);

    // Returns raw magnetometer data (optionally with user calibration offsets)
    bool BMM350::readRawMagnetometerData(float &x, float &y, float &z);

    // Calibration
    void setCalibration(float xOffset, float yOffset, float zOffset);
    void getCalibration(float &xOffset, float &yOffset, float &zOffset);

    // Power mode
    bool setPowerMode(bmm350_power_modes mode);
    bmm350_power_modes getPowerMode();
    String getPowerModeString();

    // Chip ID
    uint8_t getChipID();

    // Data rate and performance
    bool setRateAndPerformance(bmm350_data_rates rate, bmm350_performance_parameters performanceMode);
    bool setSampleRate(uint8_t rate);
    float getSampleRate();

    // Axis enable/disable
    void setEnDisAbleAxisXYZ(bmm350_x_axis_en_dis enX, bmm350_y_axis_en_dis enY, bmm350_z_axis_en_dis enZ);
    void getAxisStateXYZ(bool enAxis[3]);

    // Returns fully compensated geomagnetic data (hard/soft iron corrections)
    bmm350_mag_temp_data BMM350::readCompensatedGeomagneticData();
    // Returns heading in degrees
    float getHeadingDegree();

    // Interrupts
    bool enableInterrupt(bool enable);
    bool configureInterrupt(uint8_t latching, uint8_t polarity, uint8_t drive, uint8_t map);
    bool getInterruptStatus(uint8_t &status);
    bool setDataReadyPin(bmm350_interrupt_enable_disable modes, bmm350_intr_polarity polarity);
    bool getDataReadyState();

    // Thresholds
    void setThresholdInterrupt(uint8_t modes, int8_t threshold, bmm350_intr_polarity polarity);
    sBmm350ThresholdData_t getThresholdData();

    // Soft reset
    void softReset();

private:
    struct bmm350_dev bmm350;
    uint8_t _address;           // I2C address
    // Calibration offsets
    float calibrationX, calibrationY, calibrationZ;

    // Threshold state
    int8_t threshold = 0;
    uint8_t __thresholdMode = 3;
    sBmm350ThresholdData_t thresholdData;

    // Low-level I2C communication
    int8_t i2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len);
    int8_t i2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len);

    // Delay function for API
    void delayMs(uint32_t ms);
};