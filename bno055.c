/*
 * bno055.c
 *
 *  Created on: Oct 9, 2025
 *      Author: zeynel
 */

#include "bno055.h"

static I2C_HandleTypeDef *bno_i2c = NULL;

/* ---- BNO055 Init function --- */
BNO_StatusTypeDef bno_Init(I2C_HandleTypeDef *hi2c, BNO_MODE_t mode) {
    BNO_MODE_t _mode = mode;
	uint8_t id = 0;

    bno_SetI2CHandle(hi2c);

    if (bno_Read(BNO_CHIP_ID_ADDR, &id) != BNO_OK || id != BNO055_CHIP_ID_VALUE) {
        return BNO_ERR;
    }

    if(bno_SetMode(_mode) != BNO_OK){
    	return BNO_ERR;
    }else{
    	return BNO_OK;
    }
}

void bno_SetI2CHandle(I2C_HandleTypeDef *hi2c) {
    bno_i2c = hi2c;
}

/*---   BNO055 function to set BNO mode  -----*/
BNO_StatusTypeDef bno_SetMode(BNO_MODE_t mode) {
	if(bno_Write(BNO_OPR_MODE_ADDR, BNO_MODE_CONFIG) != BNO_OK){
		return BNO_ERR;
	}
	HAL_Delay(30);

	if(bno_Write(BNO_OPR_MODE_ADDR, mode) != BNO_OK) {
		return BNO_ERR;
	}
	HAL_Delay(30);
	return BNO_OK;
}

/*---   BNO055 function to get BNO mode  -----*/
BNO_StatusTypeDef bno_GetMode(BNO_MODE_t *pMode) {

	uint8_t modeVal = 0;

    if (pMode == NULL) return BNO_ERR;
    if (bno_Read(BNO_OPR_MODE_ADDR, &modeVal) != BNO_OK) {
        return BNO_ERR;
    }

    *pMode = (BNO_MODE_t)modeVal;

    return BNO_OK;
}

/** function to get BNO055 chip id **/
BNO_StatusTypeDef bno_Get_ChipID(uint8_t *chipID){

	if (chipID == NULL) return BNO_ERR;

	if(bno_Read(BNO_CHIP_ID_ADDR, chipID) != BNO_OK){
		return BNO_ERR;
	}
	return BNO_OK;
}

/** function to get ACC sensor chip id **/
BNO_StatusTypeDef bno_Get_ACCChipID(uint8_t *accID){

	if (accID == NULL) return BNO_ERR;

	if(bno_Read(BNO_ACC_ID_ADDR, accID) != BNO_OK){
		return BNO_ERR;
	}
	return BNO_OK;
}

/** function to get MAG sensor chip id **/
BNO_StatusTypeDef bno_Get_MAGChipID(uint8_t *magID){

	if (magID == NULL) return BNO_ERR;

	if(bno_Read(BNO_MAG_ID_ADDR, magID) != BNO_OK){
		return BNO_ERR;
	}
	return BNO_OK;
}

/** function to get GYR sensor chip id **/
BNO_StatusTypeDef bno_Get_GYRChipID(uint8_t *gyrID){

	if (gyrID == NULL) return BNO_ERR;

	if(bno_Read(BNO_GYR_ID_ADDR, gyrID) != BNO_OK){
		return BNO_ERR;
	}
	return BNO_OK;
}

/** function to get software version **/
BNO_StatusTypeDef bno_Get_SW_Rev(uint16_t *sw_rev){

	if (sw_rev == NULL) return BNO_ERR;
	uint8_t sw_rev_lsb, sw_rev_msb;
	if(bno_Read(BNO_SW_REV_ID_LSB_ADDR, &sw_rev_lsb) != BNO_OK){
		return BNO_ERR;
	}
	HAL_Delay(10);
	if(bno_Read(BNO_SW_REV_ID_MSB_ADDR, &sw_rev_msb) != BNO_OK){
		return BNO_ERR;
	}
	*sw_rev = ((uint16_t)sw_rev_msb << 8) | sw_rev_lsb;
	return BNO_OK;
}

/** function to get software version **/
BNO_StatusTypeDef bno_Get_BL_Ver(uint8_t *bl_ver){
	if (bl_ver == NULL) return BNO_ERR;
	if(bno_Read(BNO_BL_REV_ID_ADDR, bl_ver) != BNO_OK){
		return BNO_ERR;
	}
	return BNO_OK;
}

/* --- BNO055 function to write 8-bit --- */
BNO_StatusTypeDef bno_Write(uint8_t reg, uint8_t value) {
    if (bno_i2c == NULL) return BNO_ERR;

    if (HAL_I2C_Mem_Write(bno_i2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY) == HAL_OK) {
        return BNO_OK;
    } else {
        return BNO_ERR;
    }
}

/* --- BNO055 function to read 8-bit --- */
BNO_StatusTypeDef bno_Read(uint8_t reg, uint8_t *value) {
    if (bno_i2c == NULL) return BNO_ERR;

    if (HAL_I2C_Mem_Read(bno_i2c, BNO055_ADDR, reg, I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY) == HAL_OK) {
        return BNO_OK;
    } else {
        return BNO_ERR;
    }
}


