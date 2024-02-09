#ifndef ROCKETFILE
#define ROCKETFILE

#include "stm32wlxx_hal.h"
#include "RocketDefs.hpp"
#include "Accelerometer.hpp"
#include "math.h"
#include "string.h"

#define ARCHIVE_PAGE_SIZE 0x800
#define ROCKET_SETTINGS_BASE_ADDRESS 0x803F800
#define ALTIMETER_DATA_BASE_ADDRESS 0x8020000
#define ALTIMETER_ARCHIVE_PAGES 2
#define ALTIMETER_SAVE_BUFFER_SIZE 1
#define ACCELEROMETER_DATA_BASE_ADDRESS 0x802A000
#define ACCELEROMETER_ARCHIVE_PAGES 4
#define ACCELEROMETER_SAVE_BUFFER_SIZE 3
#define ARCHIVE_POSITIONS 10

class RocketFile{
public:
  RocketFile();
  HAL_StatusTypeDef SaveRocketSettings(RocketSettings *rocket_settings);
  HAL_StatusTypeDef ReadRocketSettings(RocketSettings *rocket_settings);
  HAL_StatusTypeDef OpenAltimeterArchiveWrite(uint8_t archive_position);
  HAL_StatusTypeDef WriteFlightMetadata(FlightStats *flight_stats);
  HAL_StatusTypeDef WriteAltimeterSample(float agl);
  HAL_StatusTypeDef CloseAltimeterArchive();
  void ReadFlightMetadata(uint8_t archive_position, FlightStats *flight_stats);
  bool ReadAltimeterData(uint8_t archive_position, int sample_count, int max_sample_count, uint16_t *agl);
  bool MaxAltimeterArchiveSampleIndex(uint32_t altimeter_data_archive_address, uint32_t altimeter_data_archive_base_address);
  HAL_StatusTypeDef OpenAccelerometerArchiveWrite(uint8_t archive_position);
  HAL_StatusTypeDef WriteAccelerometerSample(Accelerometer_t *accelerometer);
  HAL_StatusTypeDef CloseAccelerometerArchive();
  bool ReadAccelerometerData(uint8_t archive_position, int sample_count, int max_sample_count, Accelerometer_t *accelerometer);
  bool MaxAccelerometerArchiveSampleIndex(uint32_t accelerometer_data_archive_base_address, uint32_t accelerometer_data_archive_address);
  void UpdateArchivePosition(RocketSettings *rocket_settings);
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
  static const uint8_t archive_metadata_size_ = 11;

  HAL_StatusTypeDef ErasePages(uint32_t base_address, uint8_t pages);
};

#endif /* ROCKETFILE */
