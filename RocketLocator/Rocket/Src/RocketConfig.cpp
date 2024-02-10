#include <RocketConfig.hpp>

RocketConfig::RocketConfig(){
  //HAL_UART_Transmit(huart2, (uint8_t*)&crlf_, 2, UART_TIMEOUT);
}

RocketConfig::RocketConfig(DeviceState *device_state, RocketSettings *rocket_settings){
  device_state_ = device_state;
  rocket_settings_ = rocket_settings;
}

void RocketConfig::ProcessChar(UART_HandleTypeDef *huart2, uint8_t uart_char){
  static int char_pos = 0;
  int uart_line_len = 0;
  switch (user_interaction_state_){
  case UserInteractionState::kWaitingForCommand:
    if (((uart_char >= 'A' && uart_char <= 'Z') || (uart_char >= 'a' && uart_char <= 'z')) && char_pos < USER_INPUT_MAX_LENGTH){
      HAL_UART_Transmit(huart2, &uart_char, 1, UART_TIMEOUT);
      user_input_[char_pos++] = uart_char;
    }
    else switch (uart_char){
    case 13: // Enter key
      if (StrCmp(user_input_, config_command_, char_pos)){
        *device_state_ = DeviceState::kConfig;
        user_interaction_state_ = UserInteractionState::kConfigHome;
        deploy_mode_ = rocket_settings_->deploy_mode;
        launch_detect_altitude_ = rocket_settings_->launch_detect_altitude;
        drogue_primary_deploy_delay_ = rocket_settings_->drogue_primary_deploy_delay;
        drogue_backup_deploy_delay_ = rocket_settings_->drogue_backup_deploy_delay;
        main_primary_deploy_altitude_ = rocket_settings_->main_primary_deploy_altitude;
        main_backup_deploy_altitude_ = rocket_settings_->main_backup_deploy_altitude;
        deploy_signal_duration_ = rocket_settings_->deploy_signal_duration;
        lora_channel_ = rocket_settings_->lora_channel;
        DisplayConfigSettingsMenu(huart2);
      }
      else if (StrCmp(user_input_, data_command_, char_pos)){
        *device_state_ = DeviceState::kConfig;
        user_interaction_state_ = UserInteractionState::kGetData;
        DisplayDataMenu(huart2);
      }
      char_pos = 0;
      user_input_[0] = 0;
      break;
    case 8: // Backspace
      HAL_UART_Transmit(huart2, &uart_char, 1, UART_TIMEOUT);
      user_input_[--char_pos] = 0;
      break;
    }
    break;
  case UserInteractionState::kConfigHome:
    switch (uart_char){
    case 13: // Enter key
      rocket_settings_->deploy_mode = deploy_mode_;
      rocket_settings_->launch_detect_altitude = launch_detect_altitude_;
      rocket_settings_->drogue_primary_deploy_delay = drogue_primary_deploy_delay_;
      rocket_settings_->drogue_backup_deploy_delay = drogue_backup_deploy_delay_;
      rocket_settings_->main_primary_deploy_altitude = main_primary_deploy_altitude_;
      rocket_settings_->main_backup_deploy_altitude = main_backup_deploy_altitude_;
      rocket_settings_->deploy_signal_duration = deploy_signal_duration_;
      rocket_settings_->lora_channel = lora_channel_;
      *device_state_ = DeviceState::kConfigSavePending;
      user_interaction_state_ = UserInteractionState::kWaitingForCommand;
      uart_line_len = MakeLine(uart_line_, config_save_text_);
      break;
    case 27: // Esc key
      *device_state_ = DeviceState::kRunning;
      user_interaction_state_ = UserInteractionState::kWaitingForCommand;
      uart_line_len = MakeLine(uart_line_, config_cancel_text_);
      break;
    case 49: // 1 = Edit deploy mode
      user_interaction_state_ = UserInteractionState::kEditDeployMode;
      uart_line_len = MakeLine(uart_line_, deploy_mode_edit_text_, edit_guidance_text_, DeployModeString(deploy_mode_));
      break;
    case 50: // 2 = Edit launch detect altitude
      user_interaction_state_ = UserInteractionState::kEditLaunchDetectAltitude;
      uart_line_len = MakeLine(uart_line_, launch_detect_altitude_edit_text_, edit_guidance_text_, ToStr(launch_detect_altitude_, false));
      break;
    case 51: // 3 = Edit drogue primary deploy delay
      user_interaction_state_ = UserInteractionState::kEditDroguePrimaryDeployDelay;
      uart_line_len = MakeLine(uart_line_, drogue_primary_deploy_delay_edit_text_, edit_guidance_text_, ToStr(drogue_primary_deploy_delay_, true));
      break;
    case 52: // 4 = Edit drogue backup deploy delay
      user_interaction_state_ = UserInteractionState::kEditDrogueBackupDeployDelay;
      uart_line_len = MakeLine(uart_line_, drogue_backup_deploy_delay_edit_text_, edit_guidance_text_, ToStr(drogue_backup_deploy_delay_, true));
      break;
    case 53: // 5 = Edit main primary deploy altitude
      user_interaction_state_ = UserInteractionState::kEditMainPrimaryDeployAltitude;
      uart_line_len = MakeLine(uart_line_, main_primary_deploy_altitude_edit_text_, edit_guidance_text_, ToStr(main_primary_deploy_altitude_, false));
      break;
    case 54: // 6 = Edit main backup deploy altitude
      user_interaction_state_ = UserInteractionState::kEditMainBackupDeployAltitude;
      uart_line_len = MakeLine(uart_line_, main_backup_deploy_altitude_edit_text_, edit_guidance_text_, ToStr(main_backup_deploy_altitude_, false));
      break;
    case 55: // 7 = Edit deploy signal duration
      user_interaction_state_ = UserInteractionState::kEditDeploySignalDuration;
      uart_line_len = MakeLine(uart_line_, deploy_signal_duration_edit_text_, edit_guidance_text_, ToStr(deploy_signal_duration_, true));
      break;
    case 56: // 8 = Edit LoRa channel
      user_interaction_state_ = UserInteractionState::kEditLoraChannel;
      uart_line_len = MakeLine(uart_line_, lora_channel_edit_text_, edit_guidance_text_, ToStr(lora_channel_, false));
      break;
    }
    HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
    break;
  case UserInteractionState::kEditDeployMode:
    switch (uart_char){
    case 13: // Enter key
      user_interaction_state_ = UserInteractionState::kConfigHome;
      DisplayConfigSettingsMenu(huart2);
      break;
    case 27: // Esc key
      user_interaction_state_ = UserInteractionState::kConfigHome;
      DisplayConfigSettingsMenu(huart2);
      break;
    case 91: // [ = decrease value
      switch (deploy_mode_){
      case DeployMode::kDroguePrimaryDrogueBackup:
        deploy_mode_ = DeployMode::kDrogueBackupMainBackup;
        break;
      case DeployMode::kMainPrimaryMainBackup:
        deploy_mode_ = DeployMode::kDroguePrimaryDrogueBackup;
        break;
      case DeployMode::kDroguePrimaryMainPrimary:
        deploy_mode_ = DeployMode::kMainPrimaryMainBackup;
        break;
      case DeployMode::kDrogueBackupMainBackup:
        deploy_mode_ = DeployMode::kDroguePrimaryMainPrimary;
        break;
      }
      uart_line_len = MakeLine(uart_line_, cr_, DeployModeString(deploy_mode_));
      HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
      break;
    case 93: // [ = increase value
      switch (deploy_mode_){
      case DeployMode::kDroguePrimaryDrogueBackup:
        deploy_mode_ = DeployMode::kMainPrimaryMainBackup;
        break;
      case DeployMode::kMainPrimaryMainBackup:
        deploy_mode_ = DeployMode::kDroguePrimaryMainPrimary;
        break;
      case DeployMode::kDroguePrimaryMainPrimary:
        deploy_mode_ = DeployMode::kDrogueBackupMainBackup;
        break;
      case DeployMode::kDrogueBackupMainBackup:
        deploy_mode_ = DeployMode::kDroguePrimaryDrogueBackup;
        break;
      }
      uart_line_len = MakeLine(uart_line_, cr_, DeployModeString(deploy_mode_));
      HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
      break;
    }
    break;
  case UserInteractionState::kEditLaunchDetectAltitude:
    AdjustConfigSetting(huart2, uart_char, &launch_detect_altitude_, 50, false);
    break;
  case UserInteractionState::kEditDroguePrimaryDeployDelay:
    AdjustConfigSetting(huart2, uart_char, &drogue_primary_deploy_delay_, 20, true);
    break;
  case UserInteractionState::kEditDrogueBackupDeployDelay:
    AdjustConfigSetting(huart2, uart_char, &drogue_backup_deploy_delay_, 40, true);
    break;
  case UserInteractionState::kEditMainPrimaryDeployAltitude:
    AdjustConfigSetting(huart2, uart_char, &main_primary_deploy_altitude_, 400, false);
    break;
  case UserInteractionState::kEditMainBackupDeployAltitude:
    AdjustConfigSetting(huart2, uart_char, &main_backup_deploy_altitude_, 400, false);
    break;
  case UserInteractionState::kEditDeploySignalDuration:
    AdjustConfigSetting(huart2, uart_char, &deploy_signal_duration_, 20, true);
    break;
  case UserInteractionState::kEditLoraChannel:
    AdjustConfigSetting(huart2, uart_char, &lora_channel_, MAX_LORA_CHANNEL, false);
    break;
  case UserInteractionState::kGetData:
    if (uart_char >= '0' && uart_char <= '9')
      ExportData(huart2, uart_char - '0');
    if (uart_char == 27){ // Esc key
      *device_state_ = DeviceState::kRunning;
      user_interaction_state_ = UserInteractionState::kWaitingForCommand;
      uart_line_len = MakeLine(uart_line_, data_cancel_text_);
      HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
    }
  }
}

int RocketConfig::MakeLine(char *target, const char *source1){
  int i = 0;
  for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
    target[i] = source1[i];
  target[i] = 0;
  return i;
}

int RocketConfig::MakeLine(char *target, const char *source1, const char *source2){
  int i = 0;
  for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
    target[i] = source1[i];
  int j = 0;
  for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source2[j];
  target[i] = 0;
  return i;
}

int RocketConfig::MakeLine(char *target, const char *source1, const char *source2, const char *source3){
  int i = 0;
  for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
    target[i] = source1[i];
  int j = 0;
  for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source2[j];
  j = 0;
  for (; source3[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source3[j];
  target[i] = 0;
  return i;
}

int RocketConfig::MakeCSVExportLine(char *target, const char *source1, const char *source2){
  int i = 0;
  for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
    target[i] = source1[i];
  target[i++] = ',';
  int j = 0;
  for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source2[j];
  target[i++] = '\r';
  target[i++] = '\n';
  target[i] = 0;
  return i;
}

int RocketConfig::MakeCSVExportLine(char *target, const char *source1, const char *source2
    , const char *source3, const char *source4, const char *source5){
  int i = 0;
  for (; source1[i] != 0 && i < UART_LINE_MAX_LENGTH; i++)
    target[i] = source1[i];
  target[i++] = ',';
  int j = 0;
  for (; source2[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
  target[i] = source2[j];
  target[i++] = ',';
  j = 0;
  for (; source3[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source3[j];
  target[i] = 0;
  target[i++] = ',';
  j = 0;
  for (; source4[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source4[j];
  target[i] = 0;
  target[i++] = ',';
  j = 0;
  for (; source5[j] != 0 && i < UART_LINE_MAX_LENGTH; i++, j++)
    target[i] = source5[j];
  target[i++] = '\r';
  target[i++] = '\n';
  target[i] = 0;
  return i;
}

const char* RocketConfig::ToStr(uint16_t source, bool tenths){
  uint16_t l_source = source;
  int source_len = 0;
  if (source > 0)
    source_len = log10(source) + 1;
  else
    source_len = 1;
  char *target = new char[source_len + (tenths ? 1 : 0) + (tenths && source < 10 ? 1 : 0) + 1];
  int j = 0;
  if (tenths && source < 10)
    target[j++] = '0';
  for (int i = pow(10, source_len - 1); i > 0; i /= 10){
    if (tenths && i == 1)
      target[j++] = '.';
    int digit = l_source / i;
    l_source -= digit * i;
    target[j++] = digit + '0';
  }
  target[j] = 0;
  return (const char*) target;
}

bool RocketConfig::StrCmp(char *string1, const char *string2, int length){
  int i = 0;
  while (string1[i] == string2[i]){
    if (++i == length)
      return true;
  }
  return false;
}

void RocketConfig::DisplayConfigSettingsMenu(UART_HandleTypeDef *huart2){
  int uart_line_len = 0;
  uart_line_len = MakeLine(uart_line_, clear_screen_, config_menu_text_, crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, deploy_mode_text_, DeployModeString(deploy_mode_), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, launch_detect_altitude_text_, ToStr(launch_detect_altitude_, false), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, drogue_primary_deploy_delay_text_, ToStr(drogue_primary_deploy_delay_, true), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, drogue_backup_deploy_delay_text_, ToStr(drogue_backup_deploy_delay_, true), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, main_primary_deploy_altitude_text_, ToStr(main_primary_deploy_altitude_, false), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, main_backup_deploy_altitude_text_, ToStr(main_backup_deploy_altitude_, false), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, deploy_signal_duration_text_, ToStr(deploy_signal_duration_, true), crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, lora_channel_text_, ToStr(lora_channel_, false));
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, crlf_, crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
}

void RocketConfig::DisplayDataMenu(UART_HandleTypeDef *huart2){
  int uart_line_len = 0;
  char datetime[DATE_STRING_LENGTH] = {0};
  char archive_position[] = {'0', ')', ' ', 0};
  uart_line_len = MakeLine(uart_line_, clear_screen_, data_menu_text_, crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  uart_line_len = MakeLine(uart_line_, data_guidance_text_, crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  for (int i = 0; i < ARCHIVE_POSITIONS; i++){
    archive_position[0] = i + '0';
    rocket_file_.ReadFlightMetadata(i, &flight_stats_);
    MakeDateTime(datetime, flight_stats_.launch_date, flight_stats_.launch_time, 0, false);
    uart_line_len = MakeLine(uart_line_, archive_position, datetime, crlf_);
    HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  }
}

const char* RocketConfig::DeployModeString(DeployMode deploy_mode_value){
  switch (deploy_mode_value){
  case DeployMode::kDroguePrimaryDrogueBackup:
    return drogue_primary_drogue_backup_text_;
    break;
  case DeployMode::kMainPrimaryMainBackup:
    return main_primary_main_backup_text_;
    break;
  case DeployMode::kDroguePrimaryMainPrimary:
    return drogue_primary_main_primary_text_;
    break;
  case DeployMode::kDrogueBackupMainBackup:
    return drogue_backup_main_backup_text_;
    break;
  }
  return "\0";
}

void RocketConfig::AdjustConfigSetting(UART_HandleTypeDef *huart2, uint8_t uart_char
    , int *config_mode_setting, int max_setting_value, bool tenths){
  int uart_line_len = 0;
  switch (uart_char){
  case 13: // Enter key
    user_interaction_state_ = UserInteractionState::kConfigHome;
    DisplayConfigSettingsMenu(huart2);
    break;
  case 27: // Esc key
    user_interaction_state_ = UserInteractionState::kConfigHome;
    DisplayConfigSettingsMenu(huart2);
    break;
  case 91: // [ = decrease value
    if (*config_mode_setting > 0)
        (*config_mode_setting)--;
    break;
  case 93: // [ = increase value
    if (*config_mode_setting < max_setting_value)
        (*config_mode_setting)++;
    break;
  }
  if (uart_char == 91 || uart_char == 93){
    uart_line_len = MakeLine(uart_line_, cr_, ToStr(*config_mode_setting, tenths));
    HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  }
}

void RocketConfig::ExportData(UART_HandleTypeDef *huart2, uint8_t archive_position){
  int uart_line_len = 0;
  uart_line_len = MakeLine(uart_line_, clear_screen_, export_header_text_, crlf_);
  HAL_UART_Transmit(huart2, (uint8_t*)uart_line_, uart_line_len, UART_TIMEOUT);
  rocket_file_.ReadFlightMetadata(archive_position, &flight_stats_);
  int sample_index = 0;
  char datetime[DATE_STRING_LENGTH] = {0};
  uint16_t agl = 0;
  char s_agl[ALTIMETER_STRING_LENGTH] = {0};
  Accelerometer_t accelerometer;
  char x_accel[ACCELEROMETER_STRING_LENGTH] = {0};
  char y_accel[ACCELEROMETER_STRING_LENGTH] = {0};
  char z_accel[ACCELEROMETER_STRING_LENGTH] = {0};
  bool accelerometer_data_present = true;
  char export_line[255];
  while (rocket_file_.ReadAltimeterData(archive_position, sample_index, flight_stats_.landing_sample_count, &agl)){
    MakeDateTime(datetime, flight_stats_.launch_date, flight_stats_.launch_time, sample_index, true);
    itoa(agl, s_agl, 10);
    accelerometer_data_present = rocket_file_.ReadAccelerometerData(archive_position, sample_index
        , flight_stats_.drogue_primary_deploy_sample_count, &accelerometer);
    if (accelerometer_data_present){
      FloatToCharArray(x_accel, accelerometer.x * flight_stats_.g_range_scale, ACCELEROMETER_STRING_LENGTH);
      FloatToCharArray(y_accel, accelerometer.y * flight_stats_.g_range_scale, ACCELEROMETER_STRING_LENGTH);
      FloatToCharArray(z_accel, accelerometer.z * flight_stats_.g_range_scale, ACCELEROMETER_STRING_LENGTH);
      uart_line_len = MakeCSVExportLine(export_line, datetime, s_agl, x_accel, y_accel, z_accel);
    }
    else
      uart_line_len = MakeCSVExportLine(export_line, datetime, s_agl);
    HAL_UART_Transmit(huart2, (uint8_t*)export_line, uart_line_len, UART_TIMEOUT);
    sample_index++;
  }
}

void RocketConfig::MakeDateTime(char *target, int date, int time, int sample_count, bool fractional){
  tm sample_time;
  sample_time.tm_mday = date / 10000;
  sample_time.tm_mon = (date - date / 10000 * 10000) / 100 - 1;
  sample_time.tm_year = CENTURY + date % 100;
  sample_time.tm_hour = time / 10000;
  sample_time.tm_min = (time - time / 10000 * 10000) / 100;
  sample_time.tm_sec = (time % 100) + (sample_count < flight_stats_.drogue_primary_deploy_sample_count ? sample_count / SAMPLES_PER_SECOND
       : ceil((float)flight_stats_.drogue_primary_deploy_sample_count / SAMPLES_PER_SECOND) + sample_count - flight_stats_.drogue_primary_deploy_sample_count);
  mktime(&sample_time);
  int sample_time_length = strftime(target, DATE_STRING_LENGTH, "%Y/%m/%d %H:%M:%S", &sample_time);
  if (fractional){
    target[sample_time_length] = '.';
    if (sample_count < flight_stats_.drogue_primary_deploy_sample_count){
      target[sample_time_length + 1] = int((float)(sample_count % SAMPLES_PER_SECOND) / SAMPLES_PER_SECOND * 10) + '0';
      target[sample_time_length + 2] = int((float)(sample_count % SAMPLES_PER_SECOND) / SAMPLES_PER_SECOND * 100) % 10 + '0';
    }
    else{
      target[sample_time_length + 1] = '0';
      target[sample_time_length + 2] = '0';
    }
    target[sample_time_length + 3] = 0;
  }
  else
    target[sample_time_length] = 0;
}

void RocketConfig::FloatToCharArray(char *target, float source, uint8_t size){
  if (source >= 0)
    itoa(int(source), target, 10);
  else{
    target[0] = '-';
    itoa(int(source), target + 1, 10);
  }
  uint8_t i = 0;
  for (i = 0; target[i] != 0 && i < sizeof(target); i++);
  if (i < size - 3){
    target[i] = '.';
    itoa(abs(int((source - int(source)) * 1000)), &target[i + 1], 10);
  }
}
