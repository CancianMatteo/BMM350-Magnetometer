#include <BMM350.h>

// Prototype for communication functions
int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
static void delay_us(uint32_t period, void *intf_ptr);

BMM350::BMM350(uint8_t address)
    : _address(address), _pWire(nullptr) {}

bool BMM350::begin (TwoWire* wire){
    _pWire = wire;
    
    this->bmm350.read = i2c_read;
    this->bmm350.write = i2c_write;
    this->bmm350.delay_us = delay_us;
    this->bmm350.intf_ptr = this; // Pass the object for access to _address and _pWire

    int8_t result = bmm350_init(&(this->bmm350));
    if (result != BMM350_OK) {
        return false;
    }
    if (this->bmm350.chip_id != BMM350_CHIP_ID) {
        Wire.end();
        return false;
    }

    // Set power mode to NORMAL (required to get data)
    if (bmm350_set_powermode(BMM350_NORMAL_MODE, &bmm350) != BMM350_OK) {
        return false;
    }
    delay(10);
    // Set default data rate and performance
    if (bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_4, &bmm350) != BMM350_OK) {
        return false;
    }
    // Enable all axes by default (optional, but recommended)
    if (bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &bmm350) != BMM350_OK) {
        return false;
    }
    delay(10);

    return true;
}

/**
 * @brief Get raw ADC magnetometer values.
 * 
 * @param xRaw Output X axis value
 * @param yRaw Output Y axis value
 * @param zRaw Output Z axis value
 * @return true if successful, false otherwise
 */
bool BMM350::readRawMagnetometer (int32_t &xRaw, int32_t &yRaw, int32_t &zRaw){
    struct bmm350_raw_mag_data data;
    int8_t result = bmm350_read_uncomp_mag_temp_data(&data, &bmm350);
    if (result == BMM350_OK) {
        xRaw = data.raw_xdata;
        yRaw = data.raw_ydata;
        zRaw = data.raw_zdata;
        return true;
    }
    return false;
}

/**
 * @brief Get compensated magnetometer values.
 * 
 * @param xOut Output: X axis value
 * @param yOut Output: Y axis value
 * @param zOut Output: Z axis value
 * @return true if successful, false otherwise
 */
bool BMM350::readMagnetometerData (float &xOut, float &yOut, float &zOut){
    bmm350_mag_temp_data magTempData = {};
    // Get compensated raw data from sensor
    int8_t result = bmm350_get_compensated_mag_xyz_temp_data(&magTempData, &bmm350);
    if (result == BMM350_OK) {
        // Apply calibration offsets
        xOut = magTempData.x;
        yOut = magTempData.y;
        zOut = magTempData.z;
        return true;
    }
    return false;
}

void BMM350::setCalibration(const float new_hard_iron[3], const float new_soft_iron[3][3]) {
    for (int i = 0; i < 3; ++i) {
        hard_iron[i] = new_hard_iron[i];
        for (int j = 0; j < 3; ++j) {
            soft_iron[i][j] = new_soft_iron[i][j];
        }
    }
}

void BMM350::getCalibration(float out_hard_iron[3], float out_soft_iron[3][3]) {
    for (int i = 0; i < 3; ++i) {
        out_hard_iron[i] = hard_iron[i];
        for (int j = 0; j < 3; ++j) {
            out_soft_iron[i][j] = soft_iron[i][j];
        }
    }
}

bool BMM350::setPowerMode (bmm350_power_modes mode){
    // mode: BMM350_SUSPEND_MODE, BMM350_NORMAL_MODE, BMM350_FORCED_MODE, BMM350_FORCED_MODE_FAST
    return bmm350_set_powermode(mode, &bmm350) == BMM350_OK;
}

bmm350_power_modes BMM350::getPowerMode (){
    return static_cast<bmm350_power_modes>(bmm350.powerMode);
}

String BMM350::getPowerModeString (){
    String result;
    switch(bmm350.powerMode){
        case BMM350_SUSPEND_MODE:
            result = "Suspend mode";
            break;
        case BMM350_NORMAL_MODE:
            result = "Normal mode";
            break;
        case BMM350_FORCED_MODE:
            result = "Forced mode";
            break;
        case BMM350_FORCED_MODE_FAST:
            result = "Forced_fast mode";  
            break;
        default:
            result = "Error mode";
            break;
    }
    return result;
}

uint8_t BMM350::getChipID (){
    return bmm350.chip_id;
}

bool BMM350::enableInterrupt (bool enable){
    // Only data ready interrupt is supported in Bosch API
    return bmm350_enable_interrupt(enable ? BMM350_ENABLE_INTERRUPT : BMM350_DISABLE_INTERRUPT, &bmm350) == BMM350_OK;
}

bool BMM350::configureInterrupt (uint8_t latching, uint8_t polarity, uint8_t drive, uint8_t map){
    // Use enums from bmm350_defs.h for arguments
    return bmm350_configure_interrupt(
        (bmm350_intr_latch)latching,
        (bmm350_intr_polarity)polarity,
        (bmm350_intr_drive)drive,
        (bmm350_intr_map)map,
        &bmm350
    ) == BMM350_OK;
}

bool BMM350::getInterruptStatus (uint8_t &status){
    return bmm350_get_interrupt_status(&status, &bmm350) == BMM350_OK;
}

void BMM350::softReset (void){
    bmm350_soft_reset(&bmm350);
    bmm350_set_powermode(BMM350_SUSPEND_MODE, &bmm350);
}

bool BMM350::setRateAndPerformance (bmm350_data_rates rate, bmm350_performance_parameters performanceMode){
    bool res;
    switch (performanceMode){
        case BMM350_LOWPOWER:
            res = bmm350_set_odr_performance(rate, BMM350_NO_AVERAGING, &bmm350);
            break;
        case BMM350_REGULARPOWER:
            res = bmm350_set_odr_performance(rate, BMM350_AVERAGING_2, &bmm350);
            break;
        case BMM350_LOWNOISE:
            res = bmm350_set_odr_performance(rate, BMM350_AVERAGING_4, &bmm350);
            break;
        case BMM350_ULTRALOWNOISE:
            res = bmm350_set_odr_performance(rate, BMM350_AVERAGING_8, &bmm350);
            break;
        default:
            res = false;
            break;
    }
    return res;
}

/**
 * @brief set only the rate, keeping the averaging
 * 
 * @param rate 
 * @return true 
 * @return false 
 */
bool BMM350::setSampleRate (uint8_t rate){
    // Read current ODR and averaging register, extract current averaging (performance) bits
    uint8_t reg = 0;
    if (bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET, &reg, 1, &bmm350) != BMM350_OK) {
        return false;
    }
    uint8_t avg = reg & BMM350_AVG_MSK;
    // Set new rate, keep averaging
    reg = ((rate << BMM350_ODR_POS) & BMM350_ODR_MSK) | avg;
    // Write back the register
    if (bmm350_set_regs(BMM350_REG_PMU_CMD_AGGR_SET, &reg, 1, &bmm350) != BMM350_OK) {
        return false;
    }
    // Trigger PMU update
    uint8_t cmd = BMM350_PMU_CMD_UPD_OAE;
    if (bmm350_set_regs(BMM350_REG_PMU_CMD, &cmd, 1, &bmm350) != BMM350_OK) {
        return false;
    }
    // Write back the register
    bmm350_delay_us(BMM350_UPD_OAE_DELAY, &bmm350);
    return true;
}

/**
 * @brief return the sample rate
 * 
 * @return float [Hz]
 */
float BMM350::getSampleRate (){
    uint8_t avgODRreg = 0;
    if (bmm350_get_regs(BMM350_REG_PMU_CMD_AGGR_SET, &avgODRreg, 1, &bmm350) == BMM350_OK) {
        return -1;
    }
    uint8_t odrReg = BMM350_GET_BITS(avgODRreg, BMM350_ODR);
    switch (odrReg) {
        case BMM350_DATA_RATE_1_5625HZ: return 1.5625;
        case BMM350_DATA_RATE_3_125HZ: return 3.125;
        case BMM350_DATA_RATE_6_25HZ: return 6.25;
        case BMM350_DATA_RATE_12_5HZ: return 12.5;
        case BMM350_DATA_RATE_25HZ: return 25;
        case BMM350_DATA_RATE_50HZ: return 50;
        case BMM350_DATA_RATE_100HZ: return 100;
        case BMM350_DATA_RATE_200HZ: return 200;
        case BMM350_DATA_RATE_400HZ: return 400;
        default: return -1;
    }
}

/**
 * @brief Set the enable/disable state of the X, Y, Z axes (default enabled).
 * 
 * @param enX default: BMM350_X_EN
 * @param enY default: BMM350_Y_EN
 * @param enZ default: BMM350_Z_EN
 */
void BMM350::setEnDisAbleAxesXYZ (bmm350_x_axis_en_dis enX, bmm350_y_axis_en_dis enY, bmm350_z_axis_en_dis enZ){
    bmm350_enable_axes(enX, enY, enZ, &bmm350);
}

void BMM350::getAxesStateXYZ (bool enAxis[3]){
    // Get the configurations for ODR and performance
    uint8_t axisReg = bmm350.axis_en;
    // Read the performance status
    enAxis[0] = BMM350_GET_BITS(axisReg, BMM350_EN_X);
    enAxis[1] = BMM350_GET_BITS(axisReg, BMM350_EN_Y);
    enAxis[2] = BMM350_GET_BITS(axisReg, BMM350_EN_Z);
}

/**
 * @brief Read fully calibrated and compensated geomagnetic data with hard and soft iron corrections.
 * 
 * @param hard_iron (offsets)
 * @param soft_iron (scaling/skew)
 * @return bmm350_mag_temp_data 
 */
bmm350_mag_temp_data BMM350::readCalibratedGeomagneticData() {
    bmm350_mag_temp_data magData = {};
    bmm350_mag_temp_data magTempData = {};
    float mag_raw[3] = {0};
    float mag_calibrated[3] = {0};

    bmm350_get_compensated_mag_xyz_temp_data(&magTempData, &bmm350);

    // Apply hard iron offset correction
    mag_raw[0] = magTempData.x - hard_iron[0];
    mag_raw[1] = magTempData.y - hard_iron[1];
    mag_raw[2] = magTempData.z - hard_iron[2];

    // Apply soft iron matrix correction
    for (int i = 0; i < 3; ++i) {
        mag_calibrated[i] = soft_iron[i][0] * mag_raw[0] +
                            soft_iron[i][1] * mag_raw[1] +
                            soft_iron[i][2] * mag_raw[2];
    }

    magData.x = mag_calibrated[0];
    magData.y = mag_calibrated[1];
    magData.z = mag_calibrated[2];
    magData.temperature = magTempData.temperature;

    return magData;
}

float BMM350::getHeadingDegree (){
    // Get calibrated geomagnetic data
    bmm350_mag_temp_data magData = readCalibratedGeomagneticData();
    // Calculate heading in radians (atan2(y, x) gives angle from X axis)
    float heading = atan2(magData.y, magData.x);
    // Convert from radians to degrees
    float headingDeg = heading * (180.0f / M_PI);
    // Normalize to [0, 360)
    if (headingDeg < 0) {
        headingDeg += 360.0f;
    } else if (headingDeg >= 360.0f) {
        headingDeg -= 360.0f;
    }
    return headingDeg;
}

bool BMM350::setDataReadyPin (bmm350_interrupt_enable_disable modes, bmm350_intr_polarity polarity){
    // Get interrupt control configuration
    uint8_t regData = 0;
    if (bmm350_get_regs(BMM350_REG_INT_CTRL, &regData, 1, &bmm350) == BMM350_OK){
        regData = BMM350_SET_BITS_POS_0(regData, BMM350_INT_MODE, BMM350_PULSED);
        regData = BMM350_SET_BITS(regData, BMM350_INT_POL, polarity);
        regData = BMM350_SET_BITS(regData, BMM350_INT_OD, BMM350_INTR_PUSH_PULL);     
        regData = BMM350_SET_BITS(regData, BMM350_INT_OUTPUT_EN, BMM350_MAP_TO_PIN); 
        regData = BMM350_SET_BITS(regData, BMM350_DRDY_DATA_REG_EN, modes);
        // Finally transfer the interrupt configurations
        if (bmm350_set_regs(BMM350_REG_INT_CTRL, &regData, 1, &bmm350) != BMM350_OK){
            return false;
        }
        return true;
    }
    return false;
}

bool BMM350::getDataReadyState (){
    uint8_t drdyStatus = 0x0;
    if (bmm350_get_interrupt_status(&drdyStatus, &bmm350) != BMM350_OK)
        return false;
    return drdyStatus & 0x01;
}

void BMM350::setThresholdInterrupt (uint8_t modes, int8_t threshold, bmm350_intr_polarity polarity){
    if (modes == LOW_THRESHOLD_INTERRUPT) {
        __thresholdMode = LOW_THRESHOLD_INTERRUPT;
    } else {
        __thresholdMode = HIGH_THRESHOLD_INTERRUPT;
    }
    this->threshold = threshold;
    setDataReadyPin(BMM350_ENABLE_INTERRUPT, polarity);
}

sBmm350ThresholdData_t BMM350::getThresholdData (){
    // Clear previous threshold data
    thresholdData.mag_x = NO_DATA;
    thresholdData.mag_y = NO_DATA;
    thresholdData.mag_z = NO_DATA;
    thresholdData.interrupt_x = 0;
    thresholdData.interrupt_y = 0;
    thresholdData.interrupt_z = 0;

    // Only check if data is ready
    if (getDataReadyState()) {
        bmm350_mag_temp_data magData = readCalibratedGeomagneticData();
        int32_t thresholdValue = static_cast<int32_t>(threshold) * 16; // scale threshold

        if (__thresholdMode == LOW_THRESHOLD_INTERRUPT) {
            if (magData.x < thresholdValue) {
                thresholdData.mag_x = magData.x;
                thresholdData.interrupt_x = 1;
            }
            if (magData.y < thresholdValue) {
                thresholdData.mag_y = magData.y;
                thresholdData.interrupt_y = 1;
            }
            if (magData.z < thresholdValue) {
                thresholdData.mag_z = magData.z;
                thresholdData.interrupt_z = 1;
            }
        } else if (__thresholdMode == HIGH_THRESHOLD_INTERRUPT) {
            if (magData.x > thresholdValue) {
                thresholdData.mag_x = magData.x;
                thresholdData.interrupt_x = 1;
            }
            if (magData.y > thresholdValue) {
                thresholdData.mag_y = magData.y;
                thresholdData.interrupt_y = 1;
            }
            if (magData.z > thresholdValue) {
                thresholdData.mag_z = magData.z;
                thresholdData.interrupt_z = 1;
            }
        }
    }
    return thresholdData;
}

// --- Low-level I2C communication ---

int8_t BMM350::i2cRead(uint8_t reg_addr, uint8_t *data, uint32_t len) {
    return i2c_read(reg_addr, data, len, this);
}

int8_t BMM350::i2cWrite(uint8_t reg_addr, const uint8_t *data, uint32_t len) {
    return i2c_write(reg_addr, data, len, this);
}

void BMM350::delayMs(uint32_t ms) {
    delay(ms);
}

// --- Static I2C helpers for Bosch API ---
int8_t i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    BMM350* obj = static_cast<BMM350*>(intf_ptr);
    uint8_t address = obj->_address;
    TwoWire* wire = obj->_pWire;

    wire->beginTransmission(address);
    wire->write(reg_addr);
    if (wire->endTransmission(false) != 0) return -1;

    wire->requestFrom(address, (uint8_t)len);
    for (uint32_t i = 0; i < len && wire->available(); i++) {
        data[i] = wire->read();
    }
    return 0;
}

int8_t i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    BMM350* obj = static_cast<BMM350*>(intf_ptr);
    uint8_t address = obj->_address;
    TwoWire* wire = obj->_pWire;

    wire->beginTransmission(address);
    wire->write(reg_addr);

    for (uint32_t i = 0; i < len; i++) {
        wire->write(data[i]);
    }
    return wire->endTransmission() == 0 ? 0 : -1;
}

static void delay_us(uint32_t period, void * /*intf_ptr*/) {
    delayMicroseconds(period);
}