#ifndef ROCKET_DEFS
#define ROCKET_DEFS

#include "gpio.h"
#include "main.h"
#include "Accelerometer.hpp"

#define MILLIS_PER_SECOND 1000
#define SAMPLES_PER_SECOND 20
#define FLIGHT_DATA_ARRAY_SIZE 2 * SAMPLES_PER_SECOND
#define MAX_LORA_CHANNEL 63
#define UART_TIMEOUT 5000
#define CENTURY 100
#define ALTIMETER_SCALE 10

enum DeployMode
{
  kDroguePrimaryDrogueBackup = 1,
  kMainPrimaryMainBackup = 2,
  kDroguePrimaryMainPrimary = 3,
  kDrogueBackupMainBackup = 4
};

enum FlightStates
{
  kWaitingLaunch = 0,
  kLaunched = 1,
  kBurnout = 2,
  kNoseover = 3,
  kDroguePrimaryDeployed = 4,
  kDrogueBackupDeployed = 5,
  kMainPrimaryDeployed = 6,
  kMainBackupDeployed = 7,
  kLanded = 8
};

enum AccelerometerStates
{
  kAtRest = 0,
  kAcceleration = 1,
  kDeceleration = 2,
};

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

struct FlightStats {
  int launch_date;
  int launch_time;
  float max_altitude;
  int max_altitude_sample_count;
  float launch_detect_altitude;
  int launch_detect_sample_count;
  float burnout_altitude;
  int burnout_sample_count;
  float nose_over_altitude;
  int nose_over_sample_count;
  float drogue_primary_deploy_altitude;
  int drogue_primary_deploy_sample_count;
  float drogue_backup_deploy_altitude;
  int drogue_backup_deploy_sample_count;
  float main_primary_deploy_altitude;
  int main_primary_deploy_sample_count;
  float main_backup_deploy_altitude;
  int main_backup_deploy_sample_count;
  float landing_altitude;
  int landing_sample_count = 0;
  int sample_count = 0;
  float g_range_scale = 0;
  FlightStates flight_state = FlightStates::kWaitingLaunch;
  float agl_adjust = 0.0;
  int flight_data_array_index = 0;
  uint16_t test_data_sample_count = 0;
  float agl[FLIGHT_DATA_ARRAY_SIZE] = {0.0};
  Accelerometer_t accelerometer[FLIGHT_DATA_ARRAY_SIZE];
};

struct SensorValues {
  bool drogue_deploy_sense = GPIO_PIN_RESET;
  bool drogue_backup_deploy_sense = GPIO_PIN_RESET;
  bool main_deploy_sense = GPIO_PIN_RESET;
  bool main_backup_deploy_sense = GPIO_PIN_RESET;
};

const char displayDateFormat[] = "%02d/%02d/%d %02d:%02d:%02d";
const char fileDateFormat[] = "%Y%m%d%H%M%S";
#endif
