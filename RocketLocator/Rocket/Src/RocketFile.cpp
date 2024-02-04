#include "RocketFile.hpp"
#include "cstring"

HAL_StatusTypeDef RocketFile::SaveRocketSettings(RocketSettings *rocket_settings){
  HAL_StatusTypeDef status = HAL_OK;
  if (settings_archive_address_ == ROCKET_SETTINGS_BASE_ADDRESS)
    if(ErasePages(ROCKET_SETTINGS_BASE_ADDRESS, 1) != HAL_OK)
      return HAL_ERROR;
  if ((status = HAL_FLASH_Unlock()) != HAL_OK)
    return status;
  uint64_t* double_word = (uint64_t*)rocket_settings;
  for (uint8_t i = 0; i < ceil((float)sizeof(*rocket_settings) / sizeof(*double_word)); i++){
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, settings_archive_address_, *double_word) != HAL_OK){
      HAL_FLASH_Lock();
      return HAL_ERROR;
    }
    settings_archive_address_ += sizeof(*double_word);
    double_word++;
  }
  if ((status = HAL_FLASH_Lock()) != HAL_OK)
    return status;
  if (settings_archive_address_ >= ROCKET_SETTINGS_BASE_ADDRESS + ARCHIVE_PAGE_SIZE)
    settings_archive_address_ = ROCKET_SETTINGS_BASE_ADDRESS;
  return HAL_OK;
}

HAL_StatusTypeDef RocketFile::ReadRocketSettings(RocketSettings *rocket_settings){
  uint64_t* address;
  address = (uint64_t*)ROCKET_SETTINGS_BASE_ADDRESS;
  while (*address != 0xffffffffffffffff)
    address++;
  if (address != (uint64_t*)ROCKET_SETTINGS_BASE_ADDRESS) // Use default values if flash memory is not yet initialized
    *rocket_settings = *((RocketSettings *)(address - (uint32_t)ceil((float)sizeof(*rocket_settings) / sizeof(*address))));
  settings_archive_address_ = (uint32_t)address;
  return HAL_OK;
}

HAL_StatusTypeDef RocketFile::OpenAltimeterArchive(uint8_t archive_position){
  altimeter_data_archive_base_address_ = ALTIMETER_DATA_BASE_ADDRESS + archive_position * ALTIMETER_ARCHIVE_PAGES * ARCHIVE_PAGE_SIZE;
  altimeter_data_archive_address_ = altimeter_data_archive_base_address_;
  altimeter_data_buffer_index_ = 0;
  return ErasePages(altimeter_data_archive_base_address_, ALTIMETER_ARCHIVE_PAGES);
}

HAL_StatusTypeDef RocketFile::WriteAltimeterMetadata(AltimeterArchiveMetadata *altimeter_archive_metadata){
  if ((status = HAL_FLASH_Unlock()) != HAL_OK)
    return status;
  for (int i = 0; i < altimeter_archive_metadata_size_; i++){
    if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD
        , altimeter_data_archive_base_address_ + ALTIMETER_ARCHIVE_PAGES * ARCHIVE_PAGE_SIZE - altimeter_archive_metadata_size_
        + i * sizeof(uint64_t), *((uint64_t *)altimeter_archive_metadata + i))) != HAL_OK)
      return status;
  }
  return HAL_FLASH_Lock();
}

HAL_StatusTypeDef RocketFile::WriteAltimeterSample(float agl){
  uint16_t int16_agl = int(agl);
  if (altimeter_data_archive_address_ < altimeter_data_archive_base_address_ + ALTIMETER_ARCHIVE_PAGES * ARCHIVE_PAGE_SIZE
      - sizeof(altimeter_data_buffer_) - altimeter_archive_metadata_size_){
    std::memcpy((uint8_t *)altimeter_data_buffer_ + altimeter_data_buffer_index_, &int16_agl, sizeof(int16_agl));
    altimeter_data_buffer_index_ += sizeof(int16_agl);
    if (altimeter_data_buffer_index_ == sizeof(altimeter_data_buffer_)){
      HAL_StatusTypeDef status = HAL_OK;
      if ((status = HAL_FLASH_Unlock()) != HAL_OK)
        return status;
      for (int i = 0; i < ALTIMETER_SAVE_BUFFER_SIZE; i++){
        if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, altimeter_data_archive_address_, altimeter_data_buffer_[i])) == HAL_OK)
          altimeter_data_archive_address_ += sizeof(*altimeter_data_buffer_);
        else
          return status;
      }
      altimeter_data_buffer_index_ = 0;
      return HAL_FLASH_Lock();
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef RocketFile::CloseAltimeterArchive(){
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = (HAL_StatusTypeDef)HAL_FLASH_Unlock()) != HAL_OK)
    return status;
  if (altimeter_data_buffer_index_ > 0){
    std::memset((uint8_t *)altimeter_data_buffer_ + altimeter_data_buffer_index_, 0xff
        , sizeof(altimeter_data_buffer_) - altimeter_data_buffer_index_);
    for (int i = 0; i < ALTIMETER_SAVE_BUFFER_SIZE; i++){
      if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, altimeter_data_archive_address_, altimeter_data_buffer_[i])) == HAL_OK)
        altimeter_data_archive_address_ += sizeof(*altimeter_data_buffer_);
      else
        return status;
    }
  }
  else{
    if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, altimeter_data_archive_address_, 0xffffffffffffffff)) == HAL_OK)
      altimeter_data_archive_address_ += sizeof(*altimeter_data_buffer_);
    else
      return status;
  }
  return HAL_FLASH_Lock();
}

HAL_StatusTypeDef RocketFile::OpenAccelerometerArchive(uint8_t archive_position){
  accelerometer_data_archive_base_address_ = ACCELEROMETER_DATA_BASE_ADDRESS + archive_position * ACCELEROMETER_ARCHIVE_PAGES * ARCHIVE_PAGE_SIZE;
  accelerometer_data_archive_address_ = accelerometer_data_archive_base_address_;
  accelerometer_data_buffer_index_ = 0;
  return ErasePages(accelerometer_data_archive_base_address_, ACCELEROMETER_ARCHIVE_PAGES);
}

HAL_StatusTypeDef RocketFile::WriteAccelerometerMetadata(AccelerometerArchiveMetadata *accelerometer_archive_metadata){
  if ((status = HAL_FLASH_Unlock()) != HAL_OK)
    return status;
  for (int i = 0; i < accelerometer_archive_metadata_size_; i++){
    if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD
        , accelerometer_data_archive_base_address_ + ACCELEROMETER_ARCHIVE_PAGES * ARCHIVE_PAGE_SIZE - accelerometer_archive_metadata_size_
        + i * sizeof(uint64_t), *((uint64_t *)accelerometer_archive_metadata + i))) != HAL_OK)
      return status;
  }
  return HAL_FLASH_Lock();
}

HAL_StatusTypeDef RocketFile::WriteAccelerometerSample(Accelerometer_t *accelerometer){
  if (accelerometer_data_archive_address_ < accelerometer_data_archive_base_address_ + ACCELEROMETER_ARCHIVE_PAGES * ARCHIVE_PAGE_SIZE
      - sizeof(accelerometer_data_buffer_) - accelerometer_archive_metadata_size_){
    std::memcpy((uint8_t *)accelerometer_data_buffer_ + accelerometer_data_buffer_index_, accelerometer, sizeof(*accelerometer));
    accelerometer_data_buffer_index_ += sizeof(*accelerometer);
    if (accelerometer_data_buffer_index_ == sizeof(accelerometer_data_buffer_)){
      HAL_StatusTypeDef status = HAL_OK;
      if ((status = HAL_FLASH_Unlock()) != HAL_OK)
        return status;
      for (int i = 0; i < ACCELEROMETER_SAVE_BUFFER_SIZE; i++){
        if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, accelerometer_data_archive_address_, accelerometer_data_buffer_[i])) == HAL_OK)
          accelerometer_data_archive_address_ += sizeof(*accelerometer_data_buffer_);
        else
          return status;
      }
      accelerometer_data_buffer_index_ = 0;
      return HAL_FLASH_Lock();
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef RocketFile::CloseAccelerometerArchive(){
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = (HAL_StatusTypeDef)HAL_FLASH_Unlock()) != HAL_OK)
    return status;
  if (accelerometer_data_buffer_index_ > 0){
    std::memset((uint8_t *)accelerometer_data_buffer_ + accelerometer_data_buffer_index_, 0xff
        , sizeof(accelerometer_data_buffer_) - accelerometer_data_buffer_index_);
    for (int i = 0; i < ACCELEROMETER_SAVE_BUFFER_SIZE; i++){
      if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, accelerometer_data_archive_address_, accelerometer_data_buffer_[i])) == HAL_OK)
        accelerometer_data_archive_address_ += sizeof(*accelerometer_data_buffer_);
      else
        return status;
    }
  }
  else{
    if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, accelerometer_data_archive_address_, 0xffffffffffffffff)) == HAL_OK)
      accelerometer_data_archive_address_ += sizeof(*accelerometer_data_buffer_);
    else
      return status;
  }
  return HAL_FLASH_Lock();
}

HAL_StatusTypeDef RocketFile::ErasePages(uint32_t base_address, uint8_t pages){
  HAL_StatusTypeDef status = HAL_OK;
  if ((status = HAL_FLASH_Unlock()) != HAL_OK)
    return status;
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Page = (base_address - 0x08000000) >> 11;
  EraseInitStruct.NbPages = 1;
  uint32_t PageError;
  if ((status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError)) != HAL_OK) //Erase the Page Before a Write Operation
    return status;
  return HAL_FLASH_Lock();
}

void RocketFile::UpdateArchivePosition(uint8_t *archive_position){
  *archive_position++;
  if (*archive_position == ARCHIVE_POSITIONS)
    *archive_position = 0;
}
