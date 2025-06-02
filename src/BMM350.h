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
    bool begin(TwoWire* wire = &Wire);

    // Returns raw magnetometer data (optionally with user calibration offsets)
    bool readRawMagnetometer(int32_t &xRaw, int32_t &yRaw, int32_t &zRaw);
    bool readMagnetometerData(float &x, float &y, float &z);

    // Calibration
    void setCalibration(const float hard_iron[3], const float soft_iron[3][3]);
    void getCalibration(float hard_iron[3], float soft_iron[3][3]);

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
    void setEnDisAbleAxesXYZ(bmm350_x_axis_en_dis enX=BMM350_X_EN, bmm350_y_axis_en_dis enY=BMM350_Y_EN, bmm350_z_axis_en_dis enZ=BMM350_Z_EN);
    void getAxesStateXYZ(bool enAxis[3]);

    // Returns fully compensated geomagnetic data (hard/soft iron corrections)
    bmm350_mag_temp_data readCalibratedGeomagneticData();
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
    TwoWire* _pWire;            // Make the library more flexible and able to work with any I2C bus
    uint8_t _address;           // I2C address
    // Hard and soft iron calibration values (default values)
    float hard_iron[3] = {1.29f, 0.07f, -6.49f};
    float soft_iron[3][3] = {
        {0.974f, -0.009f, -0.005f},
        {-0.009f, 0.973f, 0.009f},
        {-0.005f, 0.009f, 1.056f}
    };

    // Threshold state
    int8_t threshold = 0;
    uint8_t __thresholdMode = 3;
    sBmm350ThresholdData_t thresholdData;

    // Low-level I2C communication
    friend int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
    friend int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);

    int8_t i2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len);
    int8_t i2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len);

    // Delay function for API
    void delayMs(uint32_t ms);
};