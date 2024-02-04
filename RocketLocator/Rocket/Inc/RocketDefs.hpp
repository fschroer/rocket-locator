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

struct SensorValues {
  bool drogue_deploy_sense = GPIO_PIN_RESET;
  bool drogue_backup_deploy_sense = GPIO_PIN_RESET;
  bool main_deploy_sense = GPIO_PIN_RESET;
  bool main_backup_deploy_sense = GPIO_PIN_RESET;
};

const char displayDateFormat[] = "%02d/%02d/%d %02d:%02d:%02d";
const char fileDateFormat[] = "%Y%m%d%H%M%S";
#endif
