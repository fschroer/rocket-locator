#include <RocketFactory.hpp>

RocketFactory::RocketFactory(){
  //rocket_file_.SaveRocketSettings(&rocket_settings_);
  rocket_file_.ReadRocketSettings(&rocket_settings_);
  if (rocket_settings_.lora_channel > MAX_LORA_CHANNEL)
  	rocket_settings_.lora_channel = 0;
  flight_manager_ = FlightManager(&rocket_settings_, &sensor_values_, &flight_stats_);
  rocket_config_ = RocketConfig(&device_state_, &rocket_settings_, &flight_stats_);
}

void RocketFactory::Begin(){
  Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
  //uint8_t hello[] = {'R', 'o', 'c', 'k', 'e', 't', 'L', 'o', 'c', 'a', 't', 'o', 'r', ' ', 'v', '1', '.', '2', '\n'};
  Radio.Send((uint8_t*)lora_startup_message_, sizeof(lora_startup_message_));
  flight_manager_.Begin();
  rocket_gps_.Begin();
  rocket_file_.OpenAltimeterArchiveWrite(rocket_settings_.archive_position);
  rocket_file_.OpenAccelerometerArchiveWrite(rocket_settings_.archive_position);
  SetDisplayDeployMode();
  HAL_Delay(1000);
  ResetDisplayDeployMode();
}

void RocketFactory::ProcessRocketEvents(){
  rocket_service_count_++;
  static int accelerometer_index = 0;
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  float x_axis = 0.0, y_axis = 0.0, z_axis = 0.0;
  flight_manager_.GetAccelerometerData(&x_axis, &y_axis, &z_axis);
  Accelerometer_t accelerometer_raw_data = flight_manager_.GetRawAccelerometerData();
  //rocket_file_.WriteAccelerometerSample(&accelerometer_raw_data);
  accelerometer_index++;
  switch (device_state_){
    case DeviceState::kRunning:
      flight_manager_.FlightService();
      if (flight_stats_.flight_state == flightStates::kLaunched){
        flight_stats_.launch_date = rocket_gps_.GetDate();
        flight_stats_.launch_time = rocket_gps_.GetTime();
        flight_stats_.g_range_scale = flight_manager_.GetGRangeScale();
        archive_opened_ = true;
      }
      if (flight_stats_.flight_state > flightStates::kWaitingLaunch && flight_stats_.flight_state < flightStates::kLanded){
        rocket_file_.WriteAltimeterSample(flight_stats_.agl[flight_stats_.sample_index]);
        if (flight_stats_.flight_state < flightStates::kDroguePrimaryDeployed)
          rocket_file_.WriteAccelerometerSample(&accelerometer_raw_data);
        else{
          if (rocket_service_count_ == 10)
            rocket_file_.WriteAltimeterSample(flight_stats_.agl[flight_stats_.sample_index]);
        }
      }
      if (flight_stats_.flight_state == flightStates::kDroguePrimaryDeployed){
        if (!accelerometer_archive_closed_){
          rocket_file_.CloseAccelerometerArchive();
          accelerometer_archive_closed_ = true;
        }
      }
      if (flight_stats_.flight_state == flightStates::kLanded){
        if (!altimeter_archive_closed_){
          rocket_file_.WriteFlightMetadata(&flight_stats_);
          rocket_file_.CloseAltimeterArchive();
          rocket_file_.UpdateArchivePosition(&rocket_settings_.archive_position);
          rocket_file_.SaveRocketSettings(&rocket_settings_);
          altimeter_archive_closed_ = true;
        }
      }
      SetDeviceState();
      if (rocket_service_count_ == 10)
        SendTelemetryData();
      if (rocket_service_count_ == 20){ // Lower frequency conserves battery.
        if (flight_stats_.flight_state == flightStates::kWaitingLaunch){ // Blink LoRa transmit LED for visual validation until liftoff
          HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        }
        rocket_service_count_ = 0;
        SendTelemetryData();
      }
      break;
    case DeviceState::kConfig:
      //ConfigDevice(x_axis, y_axis, z_axis);
      break;
    case DeviceState::kConfigSavePending:
      SetDisplayDeployMode();
      rocket_file_.SaveRocketSettings(&rocket_settings_);
      Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
      device_state_ = DeviceState::kRunning;
      ResetDisplayDeployMode();
      break;
  }
}

void RocketFactory::SetDeviceState(){
  if (flight_stats_.flight_state == flightStates::kWaitingLaunch){
    if (flight_manager_.DeviceShake()){
        //device_config_ = DeviceState::kConfig;
    }
  }
}

void RocketFactory::ConfigDevice(float x_axis, float y_axis, float z_axis){
  if (config_cycle_count_ < 60){
    if (config_cycle_count_ / 5 % 4 == 0){
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    else if (config_cycle_count_ / 5 % 4 == 1){
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    else if (config_cycle_count_ / 5 % 4 == 2){
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    }
    else if (config_cycle_count_ / 5 % 4 == 3){
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
  }
  else if (config_cycle_count_ <= 120){
    if (config_cycle_count_ / 10 % 2 == 0){
      if (x_axis < -0.9)
        DisplayDroguePrimaryDrogueBackup();
      else if (x_axis > 0.9)
        DisplayMainPrimaryMainBackup();
      else if (y_axis > 0.9)
        DisplayDrogueBackupMainBackup();
      else // default
        DisplayDroguePrimaryMainPrimary();
    }
    else if (config_cycle_count_ / 10 % 2 == 1){
      ResetDisplayDeployMode();
    }
  }
  if (config_cycle_count_ == 120){
    if (x_axis < -0.9){
      rocket_settings_.deploy_mode = DeployMode::kDroguePrimaryDrogueBackup;
      mDeployMode = DeployMode::kDroguePrimaryDrogueBackup;
    }
    else if (x_axis > 0.9){
      rocket_settings_.deploy_mode = DeployMode::kMainPrimaryMainBackup;
      mDeployMode = DeployMode::kMainPrimaryMainBackup;
    }
    else if (y_axis > 0.9){
      rocket_settings_.deploy_mode = DeployMode::kDrogueBackupMainBackup;
      mDeployMode = DeployMode::kDrogueBackupMainBackup;
    }
    else{ // default
      rocket_settings_.deploy_mode = DeployMode::kDroguePrimaryMainPrimary;
      mDeployMode = DeployMode::kDroguePrimaryMainPrimary;
    }
    rocket_file_.SaveRocketSettings(&rocket_settings_);
  }
  if (config_cycle_count_ < 120)
    config_cycle_count_++;
  else{
    config_cycle_count_ = 0;
    device_state_ = DeviceState::kRunning;
  }
}

void RocketFactory::SendTelemetryData(){
  rocket_gps_.ProcessGgaSentence();
  rocket_gps_.ProcessRmcSentence();
  rocket_gps_.SetFlightState(flight_stats_.flight_state);
  uint8_t ggaSize = rocket_gps_.GgaSize();
  uint8_t telemetryPacket[ggaSize + sizeof(float) + (SAMPLES_PER_SECOND - 1) *
                          (flight_stats_.flight_state > flightStates::kWaitingLaunch && flight_stats_.flight_state < flightStates::kDroguePrimaryDeployed)] = {0};
  rocket_gps_.GgaToPacket(telemetryPacket);
  flight_manager_.AglToPacket(telemetryPacket + ggaSize);
  Radio.Send(telemetryPacket, sizeof(telemetryPacket));
}

void RocketFactory::ProcessUART1Char(uint8_t uart_char){
  rocket_gps_.ProcessChar(uart_char);
}

void RocketFactory::ProcessUART2Char(UART_HandleTypeDef *huart2, uint8_t uart_char){
  rocket_config_.ProcessChar(huart2, uart_char);
}

void RocketFactory::DisplayDroguePrimaryDrogueBackup(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void RocketFactory::DisplayMainPrimaryMainBackup(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

void RocketFactory::DisplayDroguePrimaryMainPrimary(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

void RocketFactory::DisplayDrogueBackupMainBackup(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void RocketFactory::SetDisplayDeployMode(){
  switch (rocket_settings_.deploy_mode){
  case DeployMode::kDroguePrimaryDrogueBackup:
  	DisplayDroguePrimaryDrogueBackup();
  	break;
  case DeployMode::kMainPrimaryMainBackup:
  	DisplayMainPrimaryMainBackup();
  	break;
  case DeployMode::kDroguePrimaryMainPrimary:
  	DisplayDroguePrimaryMainPrimary();
  	break;
  case DeployMode::kDrogueBackupMainBackup:
  	DisplayDrogueBackupMainBackup();
  	break;
  }
}

void RocketFactory::ResetDisplayDeployMode(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}
