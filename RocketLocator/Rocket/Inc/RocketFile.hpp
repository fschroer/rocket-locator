#ifndef ROCKETFILE
#define ROCKETFILE

#include "stm32wlxx_hal.h"
#include <RocketDefs.hpp>
#include "math.h"

#define ROCKET_SETTINGS_ADDRESS 0x801F800

class RocketFile{
public:
	uint32_t SaveRocketSettings(RocketSettings *rocket_settings);
	void ReadRocketSettings(RocketSettings *rocket_settings);
private:
};

#endif /* ROCKETFILE */
