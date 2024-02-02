#include "RocketFile.hpp"

uint32_t RocketFile::SaveRocketSettings(RocketSettings *rocket_settings){
	HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.Page = (ROCKET_SETTINGS_ADDRESS - 0x08000000) >> 11;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)		//Erase the Page Before a Write Operation
		return HAL_FLASH_GetError();

	uint32_t address = ROCKET_SETTINGS_ADDRESS;
	uint64_t* double_word = (uint64_t*)rocket_settings;
	//double_word = (uint64_t&)rocket_settings;
	for (uint8_t i = 0; i < ceil((float)sizeof(*rocket_settings) / sizeof(*double_word)); i++){
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, *double_word) != HAL_OK)
			return HAL_FLASH_GetError();
		address += 8;
		double_word++;
	}
	HAL_FLASH_Lock();
	return HAL_OK;
}

void RocketFile::ReadRocketSettings(RocketSettings *rocket_settings){
	uint32_t address = ROCKET_SETTINGS_ADDRESS;
	if (*(__IO uint32_t *)address == 0xffffffff) // Use default values if flash memory is not yet initialized
		return;
	*rocket_settings = *(RocketSettings *)address;
}
