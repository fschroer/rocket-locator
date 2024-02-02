#ifndef ACCELEROMETER
#define ACCELEROMETER

//#include "stm32wlxx_hal.h"
//#include <stdint.h>
//#include <stdbool.h>
#include "i2c.h"
//#include "main.h"
//#include "RocketDefs.hpp"

#define MC3416_ADDR_0						0x4C //VPP connected to GND at power up
#define MC3416_ADDR_1						0x6C //VPP connected to VDD at power up
#define MC3416_CHIP_ID					0xA0
#define DEV_STAT								0x05
#define INTR_CTRL    						0x06
#define MODE         						0x07
#define SR											0x08
#define MOTION_CTRL							0x09
#define XOUT_EX_L  							0x0D
#define STATUS_2        				0x13
#define INTR_STAT_2     				0x14
#define CHIP_ID         				0x18
#define RANGE           				0x20
#define SHK_THRESH_LSB					0x46
#define PK_P2P_DUR_THRESH_LSB		0x48

typedef enum{
	MC_OK = 0,
	MC_HW_Error,
	MC_ADDR_Error,
	MC_Init_Error,
	MC_Wr_Error,
	MC_Rd_Error,
	MC_Rd_NoEqual,
	MC_ChipID_Error,
	MC_Timer_Set_Error,
}EX_Error;

class Accelerometer{
public:
	EX_Error MC3416Init();
	EX_Error GetAccelerometerValues(float *x, float *y, float *z);
private:
	typedef enum{
		g_range_2g = 0x09,
		g_range_4g = 0x19,
		g_range_8g = 0x29,
		g_range_16g = 0x39,
		g_range_12g = 0x49
	}Chip_Range_t;

	typedef enum{
		sample_rate_128_default = 0x00,
		sample_rate_256 = 0x01,
		sample_rate_512 = 0x02,
		sample_rate_1024 =0x05,
	}Chip_SampleRate_t;

	typedef struct{
		uint8_t MotionCtrl;
		uint8_t INTNCtrl;
		Chip_SampleRate_t sample_rate;
		Chip_Range_t g_range;
		uint16_t shkThresh;
		uint16_t pkP2pDurThresh;
	}mc3416_params_t;

	typedef struct{
		uint16_t addr;
		I2C_HandleTypeDef* i2c;
		mc3416_params_t params;
		uint8_t id;        /* Chip ID */
	} MC3416_HandleTypedef;

	typedef struct{
		int16_t x;
		int16_t y;
		int16_t z;
	} accelerometer_t;

	void MC3416InitDefaultParams();

  MC3416_HandleTypedef mc3416;
	float gRangeScale = (float)2 / 32768;
};

#endif  // ACCELEROMETER
