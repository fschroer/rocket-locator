#ifndef ROCKETCONFIG
#define ROCKETCONFIG

#include <RocketFile.hpp>
#include <RocketDefs.hpp>
#include "string.h"
#include "time.h"
#include "math.h"


#define UART_LINE_MAX_LENGTH 255
#define USER_INPUT_MAX_LENGTH 15
#define DATE_STRING_LENGTH 23
#define ALTIMETER_STRING_LENGTH 7
#define ACCELEROMETER_STRING_LENGTH 9
#define TEST_DEPLOY_DURATION 10 * SAMPLES_PER_SECOND

enum UserInteractionState
{
  kWaitingForCommand = 0,
  kConfigHome,
  kEditDeployChannel1Mode,
  kEditDeployChannel2Mode,
  kEditLaunchDetectAltitude,
  kEditDroguePrimaryDeployDelay,
  kEditDrogueBackupDeployDelay,
  kEditMainPrimaryDeployAltitude,
  kEditMainBackupDeployAltitude,
  kEditDeploySignalDuration,
  kEditLoraChannel,
  kEditDeviceName,
  kDataHome,
  kTestHome,
  kTestDeploy1,
  kTestDeploy2,
  kDfuHome
};

class RocketConfig{
public:
  RocketConfig();
  RocketConfig(DeviceState *device_state, RocketSettings *rocket_settings);
  void ProcessChar(uint8_t uart_char);
  void ProcessTestDeploy();
  int16_t GetTestDeployCount();
  void ResetTestDeployCount();
  void SetUserInteractionState(UserInteractionState user_interaction_state);
private:
  RocketFile rocket_file_;
  RocketSettings *rocket_settings_;
  FlightStats flight_stats_;
  UART_HandleTypeDef *huart2_;
  DeviceState *device_state_;
  UserInteractionState user_interaction_state_ = kWaitingForCommand;
  char* uart_line_ = new char[UART_LINE_MAX_LENGTH + 1];
  char* user_input_ = new char[USER_INPUT_MAX_LENGTH + 1];
  const char* clear_screen_ = "\x1b[2J\r\0";
  const char* config_command_ = "config\0";
  const char* data_command_ = "data\0";
  const char* test_command_ = "test\0";
  const char* dfu_command_ = "dfu\0";
  const char* crlf_ = "\r\n\0";
  const char* cr_ = "\r\0";
  const char* bs_ = "\b \b\0";
  const char* config_menu_intro_ = "Rocket Locator Configuration\r\n\0";
  const char* config_save_text_ = "Saved Configuration\r\n\r\n\0";
  const char* cancel_text_ = "Cancelled\r\n\r\n\0";
  const char* deployment_channel_1_mode_text_ = "0) Deployment Channel 1 Mode:\t\t\t\t\0";
  const char* deployment_channel_2_mode_text_ = "1) Deployment Channel 2 Mode:\t\t\t\t\0";
  const char* drogue_primary_text_ = "Drogue Primary\0";
  const char* drogue_backup_text_ = "Drogue Backup \0";
  const char* main_primary_text_ = "Main Primary  \0";
  const char* main_backup_text_ = "Main Backup   \0";
  const char* launch_detect_altitude_text_ = "2) Launch Detect Altitude (m):\t\t\0";
  const char* drogue_primary_deploy_delay_text_ = "3) Drogue Primary Deploy Delay (s):\t\0";
  const char* drogue_backup_deploy_delay_text_ = "4) Drogue Backup Deploy Delay (s):\t\0";
  const char* main_primary_deploy_altitude_text_ = "5) Main Primary Deploy Altitude (m):\t\0";
  const char* main_backup_deploy_altitude_text_ = "6) Main Backup Deploy Altitude (m):\t\0";
  const char* deploy_signal_duration_text_ = "7) Deploy Signal Duration (s):\t\t\0";
  const char* lora_channel_text_ = "8) Lora Channel (0-63):\t\t\t\0";
  const char* device_name_text_ = "9) Device Name:\t\t\t\t\0";
  const char* num_edit_guidance_text_ = "[ = down, ] = up. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* text_edit_guidance_text_ = "Type text. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* deploy_mode_edit_text_ = "Edit Deploy Mode\r\n\0";
  const char* launch_detect_altitude_edit_text_ = "Edit Launch Detect Altitude (m):\r\n\0";
  const char* drogue_primary_deploy_delay_edit_text_ = "Edit Drogue Primary Deploy Delay (s):\r\n\0";
  const char* drogue_backup_deploy_delay_edit_text_ = "Edit Drogue Backup Deploy Delay (s):\r\n\0";
  const char* main_primary_deploy_altitude_edit_text_ = "Edit Main Primary Deploy Altitude (m):\r\n\0";
  const char* main_backup_deploy_altitude_edit_text_ = "Edit Main Backup Deploy Altitude (m):\r\n\0";
  const char* deploy_signal_duration_edit_text_ = "Edit Deploy Signal Duration (s):\r\n\0";
  const char* lora_channel_edit_text_ = "Edit Lora Channel (0-63):\r\n\0";
  //const char* device_name_edit_text_ = "Edit Device Name:\r\n\0";

  const char* data_menu_intro_ = "Rocket Locator Data Menu\r\n\r\n\0";
  const char* data_menu_header_ = "#  Date       Time     Apogee (m) Time to Apogee (s)\r\n\0";
  const char* data_exit_text_ = "Exiting Data Menu\r\n\r\n\0";
  const char* data_guidance_text_ = "\r\nStart terminal logging and enter a valid number to retrieve CSV output of corresponding flight\r\n";
  const char* export_header_text_ = "Time, AGL, AccelX, AccelY, AccelZ\0";

  const char* test_menu_intro_ = "Rocket Locator Test Menu\r\n\r\n\0";
  const char* test_deploy1_text_ = "1) Test Deployment Channel 1\r\n\0";
  const char* test_deploy2_text_ = "2) Test Deployment Channel 2\r\n\0";
  const char* test_exit_text_ = "Exiting Data Menu\r\n\r\n\0";
  const char* test_guidance_text_ = "\r\nSelect an option and deployment test will fire in 10 seconds\r\n";
  const char* test_complete_text_ = "Test complete, exiting test mode.\r\n\r\n\0";

  const char* dfu_intro_ = "Device Firmware Upgrade\r\n\r\n\0";
  const char* dfu_guidance_text_ = "Enter to continue, Esc to cancel\r\n\r\n\0";
  const char* dfu_warning_text_ = "Warning - device will stop working until reset by administrator\r\n\0";

  const char* max_altitude_text = "Apogee: \0";
  const char* max_altitude_sample_count_text = "Apogee time: \0";
  const char* launch_detect_altitude_text = "Launch detect: \0";
  const char* launch_detect_sample_count_text = "Launch detect time: \0";
  const char* burnout_altitude_text = "Burnout: \0";
  const char* burnout_sample_count_text = "Burnout time: \0";
  const char* nose_over_altitude_text = "Noseover: \0";
  const char* nose_over_sample_count_text = "Noseover time: \0";
  const char* drogue_primary_deploy_altitude_text = "Drogue primary: \0";
  const char* drogue_primary_deploy_sample_count_text = "Drogue primary time: \0";
  const char* drogue_backup_deploy_altitude_text = "Drogue backup: \0";
  const char* drogue_backup_deploy_sample_count_text = "Drogue backup time: \0";
  const char* main_primary_deploy_altitude_text = "Main primary: \0";
  const char* main_primary_deploy_sample_count_text = "Main primary time: \0";
  const char* main_backup_deploy_altitude_text = "Main backup: \0";
  const char* main_backup_deploy_sample_count_text = "Main backup time: \0";
  const char* landing_altitude_text = "Landing: \0";
  const char* landing_sample_count_text = "Landing time: \0";

  DeployMode deployment_channel_1_mode_;
  DeployMode deployment_channel_2_mode_;
  int launch_detect_altitude_;
  int drogue_primary_deploy_delay_;
  int drogue_backup_deploy_delay_;
  int main_primary_deploy_altitude_;
  int main_backup_deploy_altitude_;
  int deploy_signal_duration_;
  int lora_channel_;
  char device_name_[DEVICE_NAME_LENGTH + 1];
  int16_t test_deploy_count_ = TEST_DEPLOY_DURATION;

  int MakeLine(char *target, const char *source1);
  int MakeLine(char *target, const char *source1, const char *source2);
  int MakeLine(char *target, const char *source1, const char *source2, const char *source3);
  void TestDeploy(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
  int MakeCSVExportLine(char *target, const char *source1, const char *source2);
  int MakeCSVExportLine(char *target, const char *source1, const char *source2, const char *source3
      , const char *source4, const char *source5);
  const char* ToStr(uint16_t source, bool tenths);
  bool StrCmp(char *string1, const char *string2, int length);
  void DisplayConfigSettingsMenu();
  const char* DeployModeString(DeployMode deploy_mode_value);
  void AdjustDeploymentChannelMode(uint8_t uart_char, DeployMode *deploy_mode);
  void AdjustConfigNumericSetting(uint8_t uart_char, int *config_mode_setting, int max_setting_value, bool tenths);
  void AdjustConfigTextSetting(uint8_t uart_char, char *config_mode_setting);
  void DisplayDataMenu();
  void DisplayTestMenu();
  void ExportData(uint8_t archive_position);
  void ExportFlightStats();
  void MakeDateTime(char *target, int date, int time, int sample_index, bool time_zone_adjust, bool fractional);
  void FloatToCharArray(char *target, float source, uint8_t size, uint8_t fraction_digits);
  void DisplayDfuMenu();
  uint8_t StartBootloader();
};

extern UART_HandleTypeDef huart2;

#endif /* ROCKETCONFIG */
