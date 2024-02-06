#ifndef ROCKET_DEFS
#define ROCKET_DEFS

#include "gpio.h"
#include "main.h"

#define MILLIS_PER_SECOND 1000
#define SAMPLES_PER_SECOND 20
#define AGL_ARRAY_SIZE 5 * SAMPLES_PER_SECOND
#define MAX_LORA_CHANNEL 63
#define UART_TIMEOUT 5000

enum DeviceState{
  kRunning = 0,
  kConfig,
  kConfigSavePending
};

enum DeployMode
{
  kDroguePrimaryDrogueBackup = 1,
  kMainPrimaryMainBackup = 2,
  kDroguePrimaryMainPrimary = 3,
  kDrogueBackupMainBackup = 4
};

enum flightStates
{
  kWaitingLaunch = 0,
  kLaunched = 1,
  kBurnout = 2,
  kNoseover = 3,
  kDroguePrimaryDeployed = 4,
  kDroguePrimarySignalOff = 5,
  kDrogueBackupDeployed = 6,
  kDrogueBackupSignalOff = 7,
  kMainPrimaryDeployed = 8,
  kMainPrimarySignalOff = 9,
  kMainBackupDeployed = 10,
  kMainBackupSignalOff = 11,
  kLanded = 12
};

struct FlightStats {
  int launch_date;
  int launch_time;
  float max_altitude;
  int max_altitude_sample_index;
  float launch_detect_altitude;
  int launch_detect_sample_index;
  float burnout_altitude;
  int burnout_sample_index;
  float nose_over_altitude;
  int nose_over_sample_index;
  float drogue_primary_deploy_altitude;
  int drogue_primary_deploy_sample_index;
  float drogue_backup_deploy_altitude;
  int drogue_backup_deploy_sample_index;
  float main_primary_deploy_altitude;
  int main_primary_deploy_sample_index;
  float main_backup_deploy_altitude;
  int main_backup_deploy_sample_index;
  float landing_altitude;
  int landing_sample_index;
  int sample_index;
  float g_range_scale;
  uint8_t flight_state = flightStates::kWaitingLaunch;
  float agl_adjust = 0.0;
  float agl[AGL_ARRAY_SIZE] = {0.0};
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
