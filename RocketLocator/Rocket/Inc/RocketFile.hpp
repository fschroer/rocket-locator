#ifndef ROCKETFILE
#define ROCKETFILE

#include "stm32wlxx_hal.h"
#include "RocketDefs.hpp"
#include "Accelerometer.hpp"
#include "math.h"

#define ARCHIVE_PAGE_SIZE 0x800
#define ROCKET_SETTINGS_BASE_ADDRESS 0x801F800
#define ALTIMETER_DATA_BASE_ADDRESS 0x8020000
#define ALTIMETER_ARCHIVE_PAGES 2
#define ALTIMETER_SAVE_BUFFER_SIZE 1
#define ACCELEROMETER_DATA_BASE_ADDRESS 0x802A000
#define ACCELEROMETER_ARCHIVE_PAGES 4
#define ACCELEROMETER_SAVE_BUFFER_SIZE 3
#define ARCHIVE_POSITIONS 10

struct RocketSettings {
  uint8_t archive_position = 0;
  DeployMode deploy_mode = kDroguePrimaryDrogueBackup;
  int launch_detect_altitude = 30; // meters
  int drogue_primary_deploy_delay = 0; // tenths of a second
  int drogue_backup_deploy_delay = 20; // tenths of a second
  int main_primary_deploy_altitude = 130; // meters
  int main_backup_deploy_altitude = 100; // meters
  int deploy_signal_duration = 10; // tenths of a second
  int lora_channel = 0;
};

struct AltimeterArchiveMetadata {
  int date;
  int time;
};

struct AccelerometerArchiveMetadata {
  int date;
  int time;
  float g_range_scale;
};

class RocketFile{
public:
  HAL_StatusTypeDef SaveRocketSettings(RocketSettings *rocket_settings);
  HAL_StatusTypeDef ReadRocketSettings(RocketSettings *rocket_settings);
  HAL_StatusTypeDef OpenAltimeterArchive(uint8_t archive_position);
  HAL_StatusTypeDef WriteAltimeterMetadata(AltimeterArchiveMetadata *altimeter_archive_metadata);
  HAL_StatusTypeDef WriteAltimeterSample(float agl);
  HAL_StatusTypeDef CloseAltimeterArchive();
  HAL_StatusTypeDef ReadAltimeterData(uint16_t *agl);
  HAL_StatusTypeDef OpenAccelerometerArchive(uint8_t archive_position);
  HAL_StatusTypeDef WriteAccelerometerMetadata(AccelerometerArchiveMetadata *accelerometer_archive_metadata);
  HAL_StatusTypeDef WriteAccelerometerSample(Accelerometer_t *accelerometer);
  HAL_StatusTypeDef CloseAccelerometerArchive();
  HAL_StatusTypeDef ReadAccelerometerData(Accelerometer_t **accelerometer);
  void UpdateArchivePosition(uint8_t *archive_position);
private:
  uint32_t settings_archive_address_ = ROCKET_SETTINGS_BASE_ADDRESS;
  uint8_t altimeter_data_buffer_index_ = 0;
  uint64_t altimeter_data_buffer_[ALTIMETER_SAVE_BUFFER_SIZE] = {0};
  uint32_t altimeter_data_archive_base_address_ = ALTIMETER_DATA_BASE_ADDRESS;
  uint32_t altimeter_data_archive_address_ = ALTIMETER_DATA_BASE_ADDRESS;
  uint8_t accelerometer_data_buffer_index_ = 0;
  uint64_t accelerometer_data_buffer_[ACCELEROMETER_SAVE_BUFFER_SIZE] = {0};
  uint32_t accelerometer_data_archive_base_address_ = ACCELEROMETER_DATA_BASE_ADDRESS;
  uint32_t accelerometer_data_archive_address_ = ACCELEROMETER_DATA_BASE_ADDRESS;
  const uint8_t altimeter_archive_metadata_size_ = ceil((float)sizeof(AltimeterArchiveMetadata) / sizeof(uint64_t)) * sizeof(uint64_t);
  const uint8_t accelerometer_archive_metadata_size_ = ceil((float)sizeof(AccelerometerArchiveMetadata) / sizeof(uint64_t)) * sizeof(uint64_t);

  HAL_StatusTypeDef ErasePages(uint32_t base_address, uint8_t pages);
};

#endif /* ROCKETFILE */
