#ifndef ROCKETCONFIG
#define ROCKETCONFIG

#include <RocketDefs.hpp>
#include "usart.h"
#include <math.h>
//#include <string>

#define UART_LINE_MAX_LENGTH 255
#define USER_INPUT_MAX_LENGTH 15

enum ConfigState
{
	kWaitingForConfigCommand = 0,
	kConfigHome,
	kEditDeployMode,
	kEditLaunchDetectAltitude,
  kEditDroguePrimaryDeployDelay,
  kEditDrogueBackupDeployDelay,
  kEditMainPrimaryDeployAltitude,
  kEditMainBackupDeployAltitude,
  kEditDeploySignalDuration,
  kEditLoraChannel
};

class RocketConfig{
public:
	RocketConfig();
	RocketConfig(DeviceState *device_state, RocketSettings *rocket_settings, FlightStats *flight_stats);
	void ProcessChar(UART_HandleTypeDef *huart2, uint8_t uart_char);
private:
	DeviceState *device_state_;
  RocketSettings *rocket_settings_;
  FlightStats *flight_stats_;

  ConfigState config_state_ = kWaitingForConfigCommand;
	char* uart_line_ = new char[UART_LINE_MAX_LENGTH + 1];
  char* user_input_ = new char[USER_INPUT_MAX_LENGTH + 1];
  const char* clear_screen_ = "\x1b[2J\0";
  const char* config_command_ = "config\0";
  const char* crlf_ = "\r\n\0";
  const char* cr_ = "\r\0";
  const char* config_menu_text_ = "\r\n\r\n\r\nRocket Locator Configuration\r\n\0";
  const char* config_save_text_ = "Saved Configuration\r\n\r\n\0";
  const char* config_cancel_text_ = "Cancelled Configuration\r\n\r\n\0";
  const char* deploy_mode_text_ = "1) Deploy Mode:\t\t\t\t\0";
  const char* drogue_primary_drogue_backup_text_ = "Drogue Primary, Drogue Backup\0";
  const char* main_primary_main_backup_text_ = "Main Primary, Main Backup    \0";
  const char* drogue_primary_main_primary_text_ = "Drogue Primary, Main Primary \0";
  const char* drogue_backup_main_backup_text_ = "Drogue Backup, Main Backup   \0";
  const char* launch_detect_altitude_text_ = "2) Launch Detect Altitude (m):\t\t\0";
  const char* drogue_primary_deploy_delay_text_ = "3) Drogue Primary Deploy Delay (s):\t\0";
  const char* drogue_backup_deploy_delay_text_ = "4) Drogue Backup Deploy Delay (s):\t\0";
  const char* main_primary_deploy_altitude_text_ = "5) Main Primary Deploy Altitude (m):\t\0";
  const char* main_backup_deploy_altitude_text_ = "6) Main Backup Deploy Altitude (m):\t\0";
  const char* deploy_signal_duration_text_ = "7) Deploy Signal Duration (s):\t\t\0";
  const char* lora_channel_text_ = "8) Lora Channel (0-63):\t\t\t\0";
  const char* edit_guidance_text_ = "[ = down, ] = up. Hit Enter to update, Esc to cancel.\r\n\0";
  const char* deploy_mode_edit_text_ = "Edit Deploy Mode\r\n\0";
  const char* launch_detect_altitude_edit_text_ = "Edit Launch Detect Altitude (m):\r\n\0";
  const char* drogue_primary_deploy_delay_edit_text_ = "Edit Drogue Primary Deploy Delay (s):\r\n\0";
  const char* drogue_backup_deploy_delay_edit_text_ = "Edit Drogue Backup Deploy Delay (s):\r\n\0";
  const char* main_primary_deploy_altitude_edit_text_ = "Edit Main Primary Deploy Altitude (m):\r\n\0";
  const char* main_backup_deploy_altitude_edit_text_ = "Edit Main Backup Deploy Altitude (m):\r\n\0";
  const char* deploy_signal_duration_edit_text_ = "Edit Deploy Signal Duration (s):\r\n\0";
  const char* lora_channel_edit_text_ = "Edit Lora Channel (0-63):\r\n\0";

	DeployMode deploy_mode_;
	int launch_detect_altitude_;
	int drogue_primary_deploy_delay_;
	int drogue_backup_deploy_delay_;
	int main_primary_deploy_altitude_;
	int main_backup_deploy_altitude_;
	int deploy_signal_duration_;
	int lora_channel_;

  int MakeLine(char *target, const char *source1);
  int MakeLine(char *target, const char *source1, const char *source2);
  int MakeLine(char *target, const char *source1, const char *source2, const char *source3);
  const char* ToStr(uint16_t source, bool tenths);
  bool StrCmp(char *string1, const char *string2, int length);
  void DisplayConfigSettings(UART_HandleTypeDef *huart2);
  const char* DeployModeString(DeployMode deploy_mode_value);
  void AdjustConfigSetting(UART_HandleTypeDef *huart2, uint8_t uart_char
  		, int *config_mode_setting, int max_setting_value, bool tenths);
};

#endif /* ROCKETCONFIG */
