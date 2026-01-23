/*
 * bno055.c
 *
 * Created on: Oct 9, 2025
 * Author: zeynel
 */

#include "bno055.h"

static I2C_HandleTypeDef *bno_i2c = NULL;

/**
 * @brief  Initializes the BNO055 sensor, checks the Chip ID, and sets the operating mode.
 * @param  hi2c: Pointer to the I2C handle (STM32 HAL).
 * @param  mode: The operating mode to configure (e.g., BNO_MODE_NDOF).
 * @retval BNO_StatusTypeDef: BNO_OK if initialization is successful, BNO_ERR otherwise.
 */
BNO_StatusTypeDef BNO_Init(I2C_HandleTypeDef *hi2c, BNO_Mode_t mode) {
    BNO_Mode_t _mode = mode;
    uint8_t id = 0;

    setI2CHandle(hi2c);

    /* Check Chip ID */
    if (bno_Read(BNO_CHIP_ID_ADDR, &id) != BNO_OK || id != BNO055_CHIP_ID_VALUE) {
        return BNO_ERR;
    }

    /* Set Operation Mode */
    if(setMode(_mode) != BNO_OK){
        return BNO_ERR;
    } else {
        return BNO_OK;
    }
}

/**
 * @brief  Sets the internal I2C handle variable to be used by read/write functions.
 * @param  hi2c: Pointer to the I2C handle.
 * @retval None
 */
void setI2CHandle(I2C_HandleTypeDef *hi2c) {
    bno_i2c = hi2c;
}

/**
 * @brief  Sets the operating mode of the BNO055 sensor.
 * Switches to CONFIG mode first if necessary.
 * @param  mode: The desired operating mode (see BNO_Mode_t).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef setMode(BNO_Mode_t mode) {
    if(bno_Write(BNO_OPR_MODE_ADDR, BNO_MODE_CONFIG) != BNO_OK){
        return BNO_ERR;
    }
    HAL_Delay(BNO_CONFIG_DELAY);

    if(bno_Write(BNO_OPR_MODE_ADDR, mode) != BNO_OK) {
        return BNO_ERR;
    }
    HAL_Delay(BNO_CONFIG_DELAY);
    return BNO_OK;
}

/**
 * @brief  Gets the current operating mode of the BNO055 sensor.
 * @param  pMode: Pointer to store the retrieved mode.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef getMode(BNO_Mode_t *pMode) {
    uint8_t modeVal = 0;

    if (pMode == NULL) return BNO_ERR;

    if (bno_Read(BNO_OPR_MODE_ADDR, &modeVal) != BNO_OK) {
        return BNO_ERR;
    }

    *pMode = (BNO_Mode_t)modeVal;

    return BNO_OK;
}

/**
 * @brief  Reads the BNO055 Chip ID.
 * @param  chipID: Pointer to store the Chip ID (should be 0xA0).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef get_ChipID(uint8_t *chipID){
    if (chipID == NULL) return BNO_ERR;

    if(bno_Read(BNO_CHIP_ID_ADDR, chipID) != BNO_OK){
        return BNO_ERR;
    }
    return BNO_OK;
}

/**
 * @brief  Reads the Accelerometer Chip ID.
 * @param  accID: Pointer to store the ACC Chip ID.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef get_ACCChipID(uint8_t *accID){
    if (accID == NULL) return BNO_ERR;

    if(bno_Read(BNO_ACC_ID_ADDR, accID) != BNO_OK){
        return BNO_ERR;
    }
    return BNO_OK;
}

/**
 * @brief  Reads the Magnetometer Chip ID.
 * @param  magID: Pointer to store the MAG Chip ID.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef get_MAGChipID(uint8_t *magID){
    if (magID == NULL) return BNO_ERR;

    if(bno_Read(BNO_MAG_ID_ADDR, magID) != BNO_OK){
        return BNO_ERR;
    }
    return BNO_OK;
}

/**
 * @brief  Reads the Gyroscope Chip ID.
 * @param  gyrID: Pointer to store the GYR Chip ID.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef get_GYRChipID(uint8_t *gyrID){
    if (gyrID == NULL) return BNO_ERR;

    if(bno_Read(BNO_GYR_ID_ADDR, gyrID) != BNO_OK){
        return BNO_ERR;
    }
    return BNO_OK;
}

/**
 * @brief  Reads the Software Revision number (MSB and LSB).
 * @param  sw_rev: Pointer to store the 16-bit software revision.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef get_SW_Rev(uint16_t *sw_rev){
    if (sw_rev == NULL) return BNO_ERR;

    uint8_t sw_rev_lsb, sw_rev_msb;

    if(bno_Read(BNO_SW_REV_ID_LSB_ADDR, &sw_rev_lsb) != BNO_OK){
        return BNO_ERR;
    }
    HAL_Delay(BNO_CONFIG_DELAY);

    if(bno_Read(BNO_SW_REV_ID_MSB_ADDR, &sw_rev_msb) != BNO_OK){
        return BNO_ERR;
    }

    *sw_rev = ((uint16_t)sw_rev_msb << 8) | sw_rev_lsb;
    return BNO_OK;
}

/**
 * @brief  Reads the Bootloader Version.
 * @param  bl_ver: Pointer to store the bootloader version.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef get_BL_Ver(uint8_t *bl_ver){
    if (bl_ver == NULL) return BNO_ERR;

    if(bno_Read(BNO_BL_REV_ID_ADDR, bl_ver) != BNO_OK){
        return BNO_ERR;
    }
    return BNO_OK;
}

/**
 * @brief  Configures the axis remap setting.
 * Switches to CONFIG mode, updates the register, and restores the previous mode.
 * @param  remapcode: The axis remap configuration (e.g., BNO_REMAP_CONFIG_P1).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef setAxisRemap(BNO_Axis_Remap_t remapcode) {
    BNO_Mode_t currentMode;

    if (getMode(&currentMode) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(BNO_MODE_CONFIG) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (bno_Write(BNO_AXIS_MAP_CONFIG_ADDR, remapcode) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(currentMode) != BNO_OK)
        return BNO_ERR;

    return BNO_OK;
}

/**
 * @brief  Configures the axis sign for the remapped axes.
 * Switches to CONFIG mode, updates the register, and restores the previous mode.
 * @param  remapsign: The axis sign configuration.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef setAxisRemapSign(BNO_Axis_Remap_Sign_t  remapsign){
    BNO_Mode_t currentMode;

    if (getMode(&currentMode) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(BNO_MODE_CONFIG) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (bno_Write(BNO_AXIS_MAP_SIGN_ADDR, remapsign) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(currentMode) != BNO_OK)
        return BNO_ERR;

    return BNO_OK;
}

/**
 * @brief  Enables or disables the use of the external crystal oscillator.
 * Using external crystal usually provides better accuracy.
 * @param  use_external: true to use external crystal, false for internal.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef setExtCrystalUse(bool use_external) {
    BNO_Mode_t currentMode;

    if (getMode(&currentMode) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(BNO_MODE_CONFIG) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (bno_Write(BNO_PAGE_ID_ADDR, 0) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    uint8_t trigger_val = use_external ? 0x80 : 0x00;
    if (bno_Write(BNO_SYS_TRIGGER_ADDR, trigger_val) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(currentMode) != BNO_OK)
        return BNO_ERR;

    return BNO_OK;
}

/**
 * @brief  Gets the system status, self-test result, and system error code.
 * @param  system_status: Pointer to store system status.
 * @param  self_test_result: Pointer to store self-test result.
 * @param  system_error: Pointer to store system error.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error){
    if (bno_i2c == NULL)
        return BNO_ERR;

    if (bno_Write(BNO_PAGE_ID_ADDR, 0) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (system_status != NULL) {
        if (bno_Read(BNO_SYS_STATUS_ADDR, system_status) != BNO_OK)
            return BNO_ERR;
    }

    if (self_test_result != NULL) {
        if (bno_Read(BNO_ST_RESULT_ADDR, self_test_result) != BNO_OK)
            return BNO_ERR;
    }

    if (system_error != NULL) {
        if (bno_Read(BNO_SYS_ERR_ADDR, system_error) != BNO_OK)
            return BNO_ERR;
    }

    HAL_Delay(BNO055_I2C_TIMEOUT);

    return BNO_OK;
}

/**
 * @brief  Gets the calibration status for all sensors.
 * Values range from 0 (uncalibrated) to 3 (fully calibrated).
 * @param  sys: Pointer to store system calibration status.
 * @param  gyro: Pointer to store gyroscope calibration status.
 * @param  accel: Pointer to store accelerometer calibration status.
 * @param  mag: Pointer to store magnetometer calibration status.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef getCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag){
    uint8_t calibrationdata;

    if(bno_Read(BNO_CALIB_STAT_ADDR, &calibrationdata) != BNO_OK)
        return BNO_ERR;

    if(sys != NULL)
        *sys = (calibrationdata >> 6) & 0x03;

    if(gyro != NULL)
        *gyro = (calibrationdata >> 4) & 0x03;

    if(accel != NULL)
        *accel = (calibrationdata >> 2) & 0x03;

    if(mag != NULL)
        *mag = calibrationdata & 0x03;

    return BNO_OK;
}

/**
 * @brief  Reads the temperature from the sensor.
 * @param  tempdata: Pointer to store the temperature (in Celsius).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef getTemperature(int8_t *tempdata){
    if (tempdata == NULL)
        return BNO_ERR;

    if(bno_Read(BNO_TEMP_ADDR, (uint8_t*)tempdata) != BNO_OK)
        return BNO_ERR;

    return BNO_OK;
}

/**
 * @brief  Writes a single byte to a specific register via I2C.
 * @param  reg: The register address to write to.
 * @param  value: The value to write.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef bno_Write(uint8_t reg, uint8_t value) {
    if (bno_i2c == NULL) return BNO_ERR;

    if (HAL_I2C_Mem_Write(bno_i2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
        return BNO_OK;
    } else {
        return BNO_ERR;
    }
}

/**
 * @brief  Reads a single byte from a specific register via I2C.
 * @param  reg: The register address to read from.
 * @param  buffer: Pointer to store the read value.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef bno_Read(uint8_t reg, uint8_t *buffer) {
    if (bno_i2c == NULL) return BNO_ERR;

    if (HAL_I2C_Mem_Read(bno_i2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, 1, BNO055_I2C_TIMEOUT) == HAL_OK) {
        return BNO_OK;
    } else {
        return BNO_ERR;
    }
}

/**
 * @brief  Reads multiple bytes from a specific register via I2C.
 * @param  reg: The starting register address.
 * @param  buffer: Pointer to the buffer to store data.
 * @param  numberofBytes: Number of bytes to read.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef bno_ReadMultiple(uint8_t reg, uint8_t *buffer, int numberofBytes) {
    if (bno_i2c == NULL) return BNO_ERR;
    if (buffer == NULL) return BNO_ERR;

    if (HAL_I2C_Mem_Read(bno_i2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buffer, numberofBytes, BNO055_I2C_TIMEOUT) == HAL_OK) {
        return BNO_OK;
    } else {
        return BNO_ERR;
    }
}

/**
 * @brief  Writes multiple bytes to a specific register via I2C.
 * @param  reg: The starting register address.
 * @param  values: Pointer to the data buffer to write.
 * @param  numberofBytes: Number of bytes to write.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef bno_WriteMultiple(uint8_t reg, uint8_t *values, int numberofBytes){
    if (bno_i2c == NULL) return BNO_ERR;
    if (values == NULL) return BNO_ERR;

    if (HAL_I2C_Mem_Write(bno_i2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, values, numberofBytes, BNO055_I2C_TIMEOUT) == HAL_OK) {
        return BNO_OK;
    } else {
        return BNO_ERR;
    }
}

/**
 * @brief  Reads acceleration data (X, Y, Z).
 * @param  accel_data: Pointer to float array (size 3) to store data in m/s^2.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readAccelleration(float* accel_data){
    if(accel_data == NULL) return BNO_ERR;
    uint8_t buffer[6];

    if (bno_ReadMultiple(BNO_ACC_DATA_X_LSB_ADDR, buffer, 6) != BNO_OK)
        return BNO_ERR;

    int16_t rawX = buffer[0] | (buffer[1] << 8);
    int16_t rawY = buffer[2] | (buffer[3] << 8);
    int16_t rawZ = buffer[4] | (buffer[5] << 8);

    accel_data[0] = rawX / 100.0f;
    accel_data[1] = rawY / 100.0f;
    accel_data[2] = rawZ / 100.0f;

    return BNO_OK;
}

/**
 * @brief  Reads magnetometer data (X, Y, Z).
 * @param  mag_data: Pointer to float array (size 3) to store data in micro-Tesla (uT).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readMagnetometer(float* mag_data){
    if(mag_data == NULL) return BNO_ERR;
    uint8_t buffer[6];

    if (bno_ReadMultiple(BNO_MAG_DATA_X_LSB_ADDR, buffer, 6) != BNO_OK)
        return BNO_ERR;

    int16_t rawX = buffer[0] | (buffer[1] << 8);
    int16_t rawY = buffer[2] | (buffer[3] << 8);
    int16_t rawZ = buffer[4] | (buffer[5] << 8);

    mag_data[0] = rawX / 16.0f;
    mag_data[1] = rawY / 16.0f;
    mag_data[2] = rawZ / 16.0f;

    return BNO_OK;
}

/**
 * @brief  Reads gyroscope data (X, Y, Z).
 * @param  gyro_data: Pointer to float array (size 3) to store data in degrees per second (dps).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readGyro(float* gyro_data){
    if(gyro_data == NULL) return BNO_ERR;
    uint8_t buffer[6];

    if (bno_ReadMultiple(BNO_GYR_DATA_X_LSB_ADDR, buffer, 6) != BNO_OK)
        return BNO_ERR;

    int16_t rawX = (int16_t)(buffer[0] | (buffer[1] << 8));
    int16_t rawY = (int16_t)(buffer[2] | (buffer[3] << 8));
    int16_t rawZ = (int16_t)(buffer[4] | (buffer[5] << 8));

    gyro_data[0] = rawX / 16.0f;
    gyro_data[1] = rawY / 16.0f;
    gyro_data[2] = rawZ / 16.0f;

    return BNO_OK;
}

/**
 * @brief  Reads Euler angles (Heading, Roll, Pitch).
 * @param  euler_data: Pointer to float array (size 3) to store data in degrees.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readEuler(float* euler_data){
    if(euler_data == NULL) return BNO_ERR;

    uint8_t buffer[6];

    if (bno_ReadMultiple(BNO_EUL_DATA_X_LSB_ADDR, buffer, 6) != BNO_OK)
        return BNO_ERR;

    int16_t rawX = (int16_t)(buffer[0] | (buffer[1] << 8)); // Heading
    int16_t rawY = (int16_t)(buffer[2] | (buffer[3] << 8)); // Roll
    int16_t rawZ = (int16_t)(buffer[4] | (buffer[5] << 8)); // Pitch

    euler_data[0] = rawX / 16.0f;
    euler_data[1] = rawY / 16.0f;
    euler_data[2] = rawZ / 16.0f;

    return BNO_OK;
}

/**
 * @brief  Reads linear acceleration data (without gravity vector).
 * @param  lin_accel_data: Pointer to float array (size 3) to store data in m/s^2.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readLinearAccelleration(float* lin_accel_data){
    if(lin_accel_data == NULL) return BNO_ERR;
    uint8_t buffer[6];

    if (bno_ReadMultiple(BNO_LIA_DATA_X_LSB_ADDR, buffer, 6) != BNO_OK)
        return BNO_ERR;

    int16_t rawX = buffer[0] | (buffer[1] << 8);
    int16_t rawY = buffer[2] | (buffer[3] << 8);
    int16_t rawZ = buffer[4] | (buffer[5] << 8);

    lin_accel_data[0] = rawX / 100.0f;
    lin_accel_data[1] = rawY / 100.0f;
    lin_accel_data[2] = rawZ / 100.0f;

    return BNO_OK;
}

/**
 * @brief  Reads gravity vector data.
 * @param  gravity_data: Pointer to float array (size 3) to store data in m/s^2.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readGravity(float* gravity_data){
    if(gravity_data == NULL) return BNO_ERR;
    uint8_t buffer[6];

    if (bno_ReadMultiple(BNO_GRV_DATA_X_LSB_ADDR, buffer, 6) != BNO_OK)
        return BNO_ERR;

    int16_t rawX = buffer[0] | (buffer[1] << 8);
    int16_t rawY = buffer[2] | (buffer[3] << 8);
    int16_t rawZ = buffer[4] | (buffer[5] << 8);

    gravity_data[0] = rawX / 100.0f;
    gravity_data[1] = rawY / 100.0f;
    gravity_data[2] = rawZ / 100.0f;

    return BNO_OK;
}

/**
 * @brief  Reads Quaternion data (W, X, Y, Z).
 * @param  quat_data: Pointer to float array (size 4) to store unitless quaternion data.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef readQuaternion(float* quat_data) {
    if(quat_data == NULL) return BNO_ERR;

    uint8_t buffer[8];

    if (bno_ReadMultiple(BNO_QUA_DATA_W_LSB_ADDR, buffer, 8) != BNO_OK)
        return BNO_ERR;

    int16_t w = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t x = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t y = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t z = (int16_t)((buffer[7] << 8) | buffer[6]);

    const float scale = 1.0f / 16384.0f;

    quat_data[0] = w * scale;
    quat_data[1] = x * scale;
    quat_data[2] = y * scale;
    quat_data[3] = z * scale;

    return BNO_OK;
}

/**
 * @brief  Reads raw sensor offset data (22 bytes) into a byte buffer.
 * Useful for saving calibration data to EEPROM/Flash.
 * @param  buffer: Pointer to a byte array (min 22 bytes).
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef getSensorOffsets_Raw(uint8_t *buffer){
    BNO_Mode_t currentMode;

    if (getMode(&currentMode) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (setMode(BNO_MODE_CONFIG) != BNO_OK)
        return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (bno_ReadMultiple(BNO_ACC_OFFSET_X_LSB_ADDR, buffer, 22) != BNO_OK)
        return BNO_ERR;

    if (setMode(currentMode) != BNO_OK)
        return BNO_ERR;

    return BNO_OK;
}

/**
 * @brief  Reads sensor calibration offsets into a structured format.
 * @param  offsets: Pointer to the BNO055_Offsets_t structure.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef getSensorOffsets(BNO055_Offsets_t *offsets) {
    BNO_Mode_t lastMode;
    uint8_t buffer[22];

    if (offsets == NULL) return BNO_ERR;

    if (getMode(&lastMode) != BNO_OK) return BNO_ERR;

    if (setMode(BNO_MODE_CONFIG) != BNO_OK) return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (bno_ReadMultiple(BNO_ACC_OFFSET_X_LSB_ADDR, buffer, 22) != BNO_OK) {
         setMode(lastMode);
         return BNO_ERR;
    }

    if (setMode(lastMode) != BNO_OK) return BNO_ERR;

    /* Accelerometer offsets */
    offsets->accel_offset_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    offsets->accel_offset_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    offsets->accel_offset_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    /* Magnetometer offsets */
    offsets->mag_offset_x   = (int16_t)((buffer[7] << 8) | buffer[6]);
    offsets->mag_offset_y   = (int16_t)((buffer[9] << 8) | buffer[8]);
    offsets->mag_offset_z   = (int16_t)((buffer[11] << 8) | buffer[10]);

    /* Gyro offsets */
    offsets->gyro_offset_x  = (int16_t)((buffer[13] << 8) | buffer[12]);
    offsets->gyro_offset_y  = (int16_t)((buffer[15] << 8) | buffer[14]);
    offsets->gyro_offset_z  = (int16_t)((buffer[17] << 8) | buffer[16]);

    /* Radius offsets */
    offsets->accel_radius   = (int16_t)((buffer[19] << 8) | buffer[18]);
    offsets->mag_radius     = (int16_t)((buffer[21] << 8) | buffer[20]);

    return BNO_OK;
}

/**
 * @brief  Writes sensor calibration offsets from a structured format to the sensor.
 * Useful for restoring calibration after power cycle.
 * @param  offsets: Pointer to the const BNO055_Offsets_t structure.
 * @retval BNO_StatusTypeDef: BNO_OK or BNO_ERR.
 */
BNO_StatusTypeDef setSensorOffsets(const BNO055_Offsets_t *offsets){
    BNO_Mode_t lastMode;

    if (offsets == NULL) return BNO_ERR;

    if (getMode(&lastMode) != BNO_OK) return BNO_ERR;

    if (setMode(BNO_MODE_CONFIG) != BNO_OK) return BNO_ERR;
    HAL_Delay(BNO_CONFIG_DELAY);

    if (bno_WriteMultiple(BNO_ACC_OFFSET_X_LSB_ADDR, (uint8_t*)offsets, 22) != BNO_OK) {
        return BNO_ERR;
    }

    if (setMode(lastMode) != BNO_OK) return BNO_ERR;

    return BNO_OK;
}
