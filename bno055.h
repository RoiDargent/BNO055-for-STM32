/*
 * bno055.h
 *
 *  Created on: Oct 9, 2025
 *      Author: zeynel
 */

#ifndef BNO055_H_
#define BNO055_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define BNO055_ADDR           (0x28 << 1)
#define BNO055_CHIP_ID_VALUE  (0xA0)

typedef enum {
	BNO_CHIP_ID_ADDR   		  = 0x00,
	BNO_ACC_ID_ADDR    		  = 0x01,
	BNO_MAG_ID_ADDR    		  = 0x02,
	BNO_GYR_ID_ADDR    		  = 0x03,
	BNO_SW_REV_ID_LSB_ADDR    = 0x04,
	BNO_SW_REV_ID_MSB_ADDR    = 0x05,
	BNO_BL_REV_ID_ADDR        = 0x06,
	BNO_PAGE_ID_ADDR          = 0x07,

	/** ACC DATA REGISTERS **/
	BNO_ACC_DATA_X_LSB_ADDR   = 0x08,
	BNO_ACC_DATA_X_MSB_ADDR   = 0x09,
	BNO_ACC_DATA_Y_LSB_ADDR   = 0x0A,
	BNO_ACC_DATA_Y_MSB_ADDR   = 0x0B,
	BNO_ACC_DATA_Z_LSB_ADDR   = 0x0C,
	BNO_ACC_DATA_Z_MSB_ADDR   = 0x0D,

	/** MAG DATA REGISTERS **/
	BNO_MAG_DATA_X_LSB_ADDR   = 0x0E,
	BNO_MAG_DATA_X_MSB_ADDR   = 0x0F,
	BNO_MAG_DATA_Y_LSB_ADDR   = 0x10,
	BNO_MAG_DATA_Y_MSB_ADDR   = 0x11,
	BNO_MAG_DATA_Z_LSB_ADDR   = 0x12,
	BNO_MAG_DATA_Z_MSB_ADDR   = 0x13,

	/** GYR DATA REGISTERS **/
	BNO_GYR_DATA_X_LSB_ADDR   = 0x14,
	BNO_GYR_DATA_X_MSB_ADDR   = 0x15,
	BNO_GYR_DATA_Y_LSB_ADDR   = 0x16,
	BNO_GYR_DATA_Y_MSB_ADDR   = 0x17,
	BNO_GYR_DATA_Z_LSB_ADDR   = 0x18,
	BNO_GYR_DATA_Z_MSB_ADDR   = 0x19,
	BNO_EUL_DATA_X_LSB_ADDR   = 0x1A,

	/* EULER DATA REGISTERS **/
	BNO_EUL_DATA_X_LSB_ADDR   = 0x1A,
	BNO_EUL_DATA_X_MSB_ADDR   = 0x1B,
	BNO_EUL_DATA_Y_LSB_ADDR   = 0x1C,
	BNO_EUL_DATA_Y_MSB_ADDR   = 0x1D,
	BNO_EUL_DATA_Z_LSB_ADDR   = 0x1E,
	BNO_EUL_DATA_Z_MSB_ADDR   = 0x1F,

	/*  QUATERNION DATA REGISTERS **/
	BNO_QUA_DATA_W_LSB_ADDR   = 0x20,
	BNO_QUA_DATA_W_MSB_ADDR   = 0x21,
	BNO_QUA_DATA_X_LSB_ADDR   = 0x22,
	BNO_QUA_DATA_X_MSB_ADDR   = 0x23,
	BNO_QUA_DATA_Y_LSB_ADDR   = 0x24,
	BNO_QUA_DATA_Y_MSB_ADDR   = 0x25,
	BNO_QUA_DATA_Z_LSB_ADDR   = 0x26,
	BNO_QUA_DATA_Z_MSB_ADDR   = 0x27,

	/** LINEAR ACCELERATION DATA REGISTERS **/
	BNO_LIA_DATA_X_LSB_ADDR   = 0x28,
	BNO_LIA_DATA_X_MSB_ADDR   = 0x29,
	BNO_LIA_DATA_Y_LSB_ADDR   = 0x2A,
	BNO_LIA_DATA_Y_MSB_ADDR   = 0x2B,
	BNO_LIA_DATA_Z_LSB_ADDR   = 0x2C,
	BNO_LIA_DATA_Z_LSB_ADDR   = 0x2D,

	/** GRAVITY DATA REGISTERS **/
	BNO_GRV_DATA_X_LSB_ADDR   = 0x2E,
	BNO_GRV_DATA_X_MSB_ADDR   = 0x2F,
	BNO_GRV_DATA_Y_LSB_ADDR   = 0x30,
	BNO_GRV_DATA_Y_MSB_ADDR   = 0x31,
	BNO_GRV_DATA_Z_LSB_ADDR   = 0x32,
	BNO_GRV_DATA_Z_MSB_ADDR   = 0x33,

	/** TEMPERATURE DATA REGISTER **/
	BNO_TEMP_ADDR             = 0x34,

	/** STATUS DATA REGISTERS **/
	BNO_CALIB_STAT_ADDR 	  = 0x35,
	BNO_ST_RESULT_ADDR 		  = 0x36,
	BNO_INT_STA_ADDR 	  	  = 0x37,

	BNO_SYS_CLK_STATUS_ADDR   = 0x38,
	BNO_SYS_STATUS_ADDR 	  = 0x39,
	BNO_SYS_ERR_ADDR 		  = 0x3A,

	/** UNIT SELECTION REGISTER **/
	BNO_UNIT_SEL_ADDR 		  = 0x3B,

	/** MODE REGISTERS **/
	BNO_OPR_MODE_ADDR 		  = 0X3D,
	BNO_PWR_MODE_ADDR 		  = 0X3E,

	BNO_SYS_TRIGGER_ADDR 	  = 0x3F,
	BNO_TEMP_SOURCE_ADDR	  = 0x40,

	/** AXIS REMAP REGISTERS **/
	BNO_AXIS_MAP_CONFIG_ADDR  = 0x41,
	BNO_AXIS_MAP_SIGN_ADDR 	  = 0x42,

	/** ACC OFFSET DATA REGISRTERS **/
	BNO_ACC_OFFSET_X_LSB_ADDR = 0x55,
	BNO_ACC_OFFSET_X_MSB_ADDR = 0x56,
	BNO_ACC_OFFSET_Y_LSB_ADDR = 0x57,
	BNO_ACC_OFFSET_Y_MSB_ADDR = 0x58,
	BNO_ACC_OFFSET_Z_LSB_ADDR = 0x59,
	BNO_ACC_OFFSET_Z_MSB_ADDR = 0x5A,

	/** MAG OFFSET DATA REGISTERS **/
	BNO_MAG_OFFSET_X_LSB_ADDR = 0x5B,
	BNO_MAG_OFFSET_X_MSB_ADDR = 0x5C,
	BNO_MAG_OFFSET_Y_LSB_ADDR = 0x5D,
	BNO_MAG_OFFSET_Y_MSB_ADDR = 0x5E,
	BNO_MAG_OFFSET_Z_LSB_ADDR = 0x5F,
	BNO_MAG_OFFSET_Z_MSB_ADDR = 0x60,

	/** GYR OFFSET DATA REGISTERS **/
	BNO_GYR_OFFSET_X_LSB_ADDR = 0x61,
	BNO_GYR_OFFSET_X_MSB_ADDR = 0x62,
	BNO_GYR_OFFSET_Y_LSB_ADDR = 0x63,
	BNO_GYR_OFFSET_Y_MSB_ADDR = 0x64,
	BNO_GYR_OFFSET_Z_LSB_ADDR = 0x65,
	BNO_GYR_OFFSET_Z_MSB_ADDR = 0x66,

	/** ACC RADIUS DATA REGISTERS **/
	BNO_ACC_RADIUS_LSB_ADDR   = 0x67,
	BNO_ACC_RADIUS_MSB_ADDR   = 0x68,

	/** ACC RADIUS DATA REGISTERS **/
	BNO_MAG_RADIUS_LSB_ADDR   = 0x69,
	BNO_MAG_RADIUS_MSB_ADDR   = 0x6A
}BNO_Regs_t;

/** Axis Remap Configurations **/
typedef enum {
	BNO_REMAP_CONFIG_P0 = 0x21,
	BNO_REMAP_CONFIG_P1 = 0x24, // default positioning
	BNO_REMAP_CONFIG_P2 = 0x24,
	BNO_REMAP_CONFIG_P3 = 0x21,
	BNO_REMAP_CONFIG_P4 = 0x24,
	BNO_REMAP_CONFIG_P5 = 0x21,
	BNO_REMAP_CONFIG_P6 = 0x21,
	BNO_REMAP_CONFIG_P7 = 0x24
}BNO_Axis_Remap_t;

/** Axis Remap Sign Configurations **/
typedef enum {
  REMAP_SIGN_P0 = 0x04,
  REMAP_SIGN_P1 = 0x00, // default
  REMAP_SIGN_P2 = 0x06,
  REMAP_SIGN_P3 = 0x02,
  REMAP_SIGN_P4 = 0x03,
  REMAP_SIGN_P5 = 0x01,
  REMAP_SIGN_P6 = 0x07,
  REMAP_SIGN_P7 = 0x05
} BNO_Axis_Remap_Sign_t;

/* ---- BNO055 Types of Modes ---*/
typedef enum {
     BNO_MODE_CONFIG        = 0x00,
     BNO_MODE_ACCONLY       = 0x01,
     BNO_MODE_MAGONLY       = 0x02,
     BNO_MODE_GYRONLY       = 0x03,
     BNO_MODE_ACCMAG        = 0x04,
     BNO_MODE_ACCGYRO       = 0x05,
     BNO_MODE_MAGGYRO       = 0x06,
     BNO_MODE_AMG           = 0x07,
     BNO_MODE_IMUPLUS       = 0x08,
     BNO_MODE_COMPASS       = 0x09,
     BNO_MODE_M4G           = 0x0A,
     BNO_MODE_NDOF_FMC_OFF  = 0x0B,
     BNO_MODE_NDOF          = 0x0C
} BNO_Mode_t;

/* --- BNO055 custom return status --- */
typedef enum {
    BNO_OK,
    BNO_ERR
} BNO_StatusTypeDef;

BNO_StatusTypeDef bno_Init(I2C_HandleTypeDef *hi2c, BNO_MODE_t mode);
void bno_SetI2CHandle(I2C_HandleTypeDef *hi2c);

BNO_StatusTypeDef bno_SetMode(BNO_MODE_t mode);
BNO_StatusTypeDef bno_GetMode(BNO_MODE_t *pMode);

BNO_StatusTypeDef bno_Get_ChipID(uint8_t *chipID);
BNO_StatusTypeDef bno_Get_ACCChipID(uint8_t *accID);
BNO_StatusTypeDef bno_Get_MAGChipID(uint8_t *magID);
BNO_StatusTypeDef bno_Get_GYRChipID(uint8_t *gyrID);

BNO_StatusTypeDef bno_Get_SW_Rev(uint16_t *sw_rev);
BNO_StatusTypeDef bno_Get_BL_Ver(uint8_t *bl_ver);

BNO_StatusTypeDef bno_SetAxisRemap(BNO_Axis_Remap_t remapcode);
BNO_StatusTypeDef bno_SetAxisRemapSign(BNO_Axis_Remap_Sign_t  remapsign);

BNO_StatusTypeDef bno_setExtCrystalUse(bool use_external);

BNO_StatusTypeDef bno_GetSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error);
BNO_StatusTypeDef bno_getCalibraiton(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);

BNO_StatusTypeDef bno_getTemperature(int8_t *tempdata)

BNO_StatusTypeDef bno_Write(uint8_t reg, uint8_t value);
BNO_StatusTypeDef bno_Read(uint8_t reg, uint8_t *buffer);
BNO_StatusTypeDef bno_ReadMultiple(uint8_t reg, uint8_t *buffer, int numberofBytes);

BNO_StatusTypeDef bno_readAccelleration(float* accel_data);
BNO_StatusTypeDef bno_readMagnetometer(float* mag_data);
BNO_StatusTypeDef bno_readGyro(float* gyro_data);
BNO_StatusTypeDef bno_readEuler(float* euler_data);
BNO_StatusTypeDef bno_readLinearAccelleration(float* lin_accel_data);
BNO_StatusTypeDef bno_readGravity(float* gravity_data);
BNO_StatusTypeDef bno_readQuaternion(float* quat_data);


#endif /* BNO055_H_ */
