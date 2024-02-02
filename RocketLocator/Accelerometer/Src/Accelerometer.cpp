//#include "sys_app.h"
#include "../../Accelerometer/Inc/Accelerometer.hpp"

void Accelerometer::MC3416InitDefaultParams(){
	mc3416.params.MotionCtrl = 0x00;
	mc3416.params.INTNCtrl = 0x00;
	mc3416.params.g_range = g_range_16g;
	mc3416.params.sample_rate = sample_rate_128_default;
	mc3416.params.shkThresh = 0x0000;
	mc3416.params.pkP2pDurThresh = 0x0000;
}

EX_Error Accelerometer::MC3416Init(){
	mc3416.addr = MC3416_ADDR_0;
	mc3416.i2c = &hi2c2;
	MC3416InitDefaultParams();
	if (HAL_I2C_Mem_Read(mc3416.i2c, mc3416.addr << 1, CHIP_ID, 1, &mc3416.id, 1, 5000) != HAL_OK)
		return MC_Rd_Error;
	if (mc3416.id != MC3416_CHIP_ID)
		return MC_ChipID_Error;

	switch ((int)mc3416.params.g_range){
		case g_range_2g:
			gRangeScale = (float)2 / 32768;
			break;
		case g_range_4g:
			gRangeScale = (float)4 / 32768;
			break;
		case g_range_8g:
			gRangeScale = (float)8 / 32768;
			break;
		case g_range_12g:
			gRangeScale = (float)12 / 32768;
			break;
		case g_range_16g:
			gRangeScale = (float)16 / 32768;
			break;
	}
	uint8_t mc3416Mode = 0x10; //Standby mode
	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, MODE, 1, &mc3416Mode, 1, 10000) != HAL_OK)
		return MC_Wr_Error;
  HAL_Delay(1); //Allow time for mode change
	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, INTR_CTRL, 1, &mc3416.params.INTNCtrl, 1, 10000) != HAL_OK)
		return MC_Wr_Error;
	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, MOTION_CTRL, 1, &mc3416.params.MotionCtrl, 1, 10000) != HAL_OK)
		return MC_Wr_Error;
	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, RANGE, 1, (uint8_t*)&mc3416.params.g_range, 1, 10000) != HAL_OK)
		return MC_Wr_Error;
	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, SR, 1, (uint8_t*)&mc3416.params.sample_rate, 1, 10000) != HAL_OK)
		return MC_Wr_Error;
//	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, SHK_THRESH_LSB, 1, (uint8_t*)&mc3416.params.shkThresh, 2, 10000) != HAL_OK)
//		return MC_Wr_Error;
//	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, PK_P2P_DUR_THRESH_LSB, 1, (uint8_t*)&mc3416.params.pkP2pDurThresh, 2, 10000) != HAL_OK)
//		return MC_Wr_Error;
	mc3416Mode = 0x11; //Wake mode
	if (HAL_I2C_Mem_Write(mc3416.i2c, mc3416.addr << 1, MODE, 1, &mc3416Mode, 1, 10000) != HAL_OK)
		return MC_Wr_Error;
  HAL_Delay(10); //Allow time for mode change
	return MC_OK;
}

EX_Error Accelerometer::GetAccelerometerValues(float *x, float *y, float *z){
//	uint8_t devStatus;
//	uint16_t shk;
//	uint16_t dur;
//	uint8_t accelInt;
//	if (HAL_I2C_Mem_Read(mc3416.i2c, mc3416.addr << 1, DEV_STAT, 1, &devStatus, 1, 5000) != HAL_OK)
//		return MC_Rd_Error;
//	if (HAL_I2C_Mem_Read(mc3416.i2c, mc3416.addr << 1, SHK_THRESH_LSB, 1, (uint8_t *)&shk, 2, 5000) != HAL_OK)
//		return MC_Rd_Error;
//	if (HAL_I2C_Mem_Read(mc3416.i2c, mc3416.addr << 1, PK_P2P_DUR_THRESH_LSB, 1, (uint8_t *)&dur, 2, 5000) != HAL_OK)
//		return MC_Rd_Error;
//	if (HAL_I2C_Mem_Read(mc3416.i2c, mc3416.addr << 1, INTR_STAT_2, 1, &accelInt, 1, 5000) != HAL_OK)
//		return MC_Rd_Error;
	accelerometer_t accelerometer;
	if (HAL_I2C_Mem_Read(mc3416.i2c, mc3416.addr << 1, XOUT_EX_L, 1, (uint8_t*)&accelerometer, 6, 5000) != HAL_OK)
		return MC_Rd_Error;
	*x = accelerometer.x * gRangeScale;
	*y = accelerometer.y * gRangeScale;
	*z = accelerometer.z * gRangeScale;
	return MC_OK;
}
