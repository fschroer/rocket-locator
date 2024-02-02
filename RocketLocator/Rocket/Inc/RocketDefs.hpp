#ifndef ROCKET_DEFS
#define ROCKET_DEFS

#include "gpio.h"
#include "main.h"

#define MILLIS_PER_SECOND 1000
#define SAMPLES_PER_SECOND 20
#define AGL_ARRAY_SIZE 4 * 60 * SAMPLES_PER_SECOND
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
  kWaitingLDA = 0,
  kReachedLDA = 1,
	kMachLockoutEntered = 2,
  kMachLockoutReleased = 3,
	kBurnout = 4,
  kNoseover = 5,
  kDroguePrimaryDeployed = 6,
  kDroguePrimarySignalOff = 7,
	kDrogueBackupDeployed = 8,
	kDrogueBackupSignalOff = 9,
  kMainPrimaryDeployed = 10,
  kMainPrimarySignalOff = 11,
  kMainBackupDeployed = 12,
  kMainBackupSignalOff = 13,
  kLanded = 14
};

struct RocketSettings {
  DeployMode deploy_mode = kDroguePrimaryDrogueBackup;
  int launch_detect_altitude = 30; // meters
  int drogue_primary_deploy_delay = 0; // tenths of a second
  int drogue_backup_deploy_delay = 20; // tenths of a second
  int main_primary_deploy_altitude = 130; // meters
  int main_backup_deploy_altitude = 100; // meters
  int deploy_signal_duration = 10; // tenths of a second
  int lora_channel = 0;
};

struct SensorValues {
  bool drogue_deploy_sense = GPIO_PIN_RESET;
  bool drogue_backup_deploy_sense = GPIO_PIN_RESET;
  bool main_deploy_sense = GPIO_PIN_RESET;
  bool main_backup_deploy_sense = GPIO_PIN_RESET;
};

struct FlightStats {
  //bool armed; //deployment system enabled state
  //bool prevArmed;
	uint8_t flight_state = flightStates::kWaitingLDA;
  //int prevFlightState = flightStates::WAITING_LDA;
  float agl_adjust = 0.0;
  float max_altitude;
  int max_altitude_time;
  float launch_detect_altitude;
  int launch_detect_time;
  float mach_lockout_entered_altitude;
  int mach_lockout_entered_time;
  float burnout_altitude;
  int burnout_time;
  float mach_lockout_released_altitude;
  int mach_lockout_released_time;
  float nose_over_altitude;
  int nose_over_time;
  float drogue_primary_deploy_altitude;
  int drogue_primary_deploy_time;
  float drogue_backup_deploy_altitude;
  int drogue_backup_deploy_time;
  float main_primary_deploy_altitude;
  int main_primary_deploy_time;
  float main_backup_deploy_altitude;
  int main_backup_deploy_time;
  float land_altitude;
  int land_time;
  int sample_index;
  float agl[AGL_ARRAY_SIZE] = {0.0};
};

const char displayDateFormat[] = "%02d/%02d/%d %02d:%02d:%02d";
const char fileDateFormat[] = "%Y%m%d%H%M%S";
#endif
