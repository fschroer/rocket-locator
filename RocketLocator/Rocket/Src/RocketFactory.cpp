#include <RocketFactory.hpp>

RocketFactory::RocketFactory(){
  //rocket_file_.SaveRocketSettings(&rocket_settings_);
  rocket_file_.ReadRocketSettings(&rocket_settings_);
  if (rocket_settings_.lora_channel > MAX_LORA_CHANNEL)
  	rocket_settings_.lora_channel = 0;
  flight_manager_ = FlightManager(&rocket_settings_, &sensor_values_, &flight_stats_);
  rocket_config_ = RocketConfig(&device_state_, &rocket_settings_);
}

void RocketFactory::Begin(){
  Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
  Radio.Send((uint8_t*)lora_startup_message_, strlen(lora_startup_message_));
  flight_manager_.Begin();
  rocket_gps_.Begin();
  rocket_file_.OpenAltimeterArchiveWrite(rocket_settings_.archive_position);
  rocket_file_.OpenAccelerometerArchiveWrite(rocket_settings_.archive_position);
  SetDisplayDeployMode();
  HAL_Delay(1000);
  ResetDisplayDeployMode();
}

void RocketFactory::ProcessRocketEvents(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
  switch (device_state_){
    case DeviceState::kRunning:
      rocket_service_count_++;
      flight_manager_.FlightService();
      if (flight_stats_.flight_state >= FlightStates::kLaunched && !archive_opened_){
        for (int i = flight_stats_.flight_data_array_index - LAUNCH_LOOKBACK_SAMPLES + 1; i < flight_stats_.flight_data_array_index; i++){
          rocket_file_.WriteAltimeterSample(flight_stats_.agl[i]);
          rocket_file_.WriteAccelerometerSample(&flight_stats_.accelerometer[i]);
        }
        archive_opened_ = true;
      }
      if (flight_stats_.flight_state >= FlightStates::kLaunched && rocket_gps_.GPSDatestampValid() && !datestamp_saved_){
        flight_stats_.launch_date = rocket_gps_.GetDate(); //low priority to do: adjust date / time if received valid after launch
        flight_stats_.launch_time = rocket_gps_.GetTime();
        datestamp_saved_ = true;
      }
      if (flight_stats_.flight_state > FlightStates::kWaitingLaunch && flight_stats_.flight_state < FlightStates::kLanded){
        if (flight_stats_.flight_state < FlightStates::kDroguePrimaryDeployed){
          rocket_file_.WriteAltimeterSample(flight_stats_.agl[flight_stats_.flight_data_array_index]);
          rocket_file_.WriteAccelerometerSample(&flight_stats_.accelerometer[flight_stats_.flight_data_array_index]);
          flight_stats_.sample_count++;
        }
        else if (rocket_service_count_ == SAMPLES_PER_SECOND){
            rocket_file_.WriteAltimeterSample(flight_stats_.agl[flight_stats_.flight_data_array_index]);
            flight_stats_.sample_count++;
        }
      }
      if (flight_stats_.flight_state >= FlightStates::kDroguePrimaryDeployed && !accelerometer_archive_closed_){
        rocket_file_.WriteAccelerometerSample(&flight_stats_.accelerometer[flight_stats_.flight_data_array_index]);
        rocket_file_.CloseAccelerometerArchive();
        accelerometer_archive_closed_ = true;
      }
      if (flight_stats_.flight_state == FlightStates::kLanded){
        if (!datestamp_saved_){
          flight_stats_.launch_date = rocket_gps_.GetDate();
          flight_stats_.launch_time = rocket_gps_.GetTime();
        }
        if (!altimeter_archive_closed_){
          rocket_file_.WriteFlightMetadata(&flight_stats_);
          rocket_file_.WriteAltimeterSample(flight_stats_.agl[flight_stats_.flight_data_array_index]);
          rocket_file_.CloseAltimeterArchive();
          rocket_file_.UpdateArchivePosition(&rocket_settings_);
          rocket_file_.SaveRocketSettings(&rocket_settings_);
          altimeter_archive_closed_ = true;
        }
      }
//      if (rocket_service_count_ == SAMPLES_PER_SECOND / 2)
//        if (flight_stats_.flight_state > flightStates::kWaitingLaunch && flight_stats_.flight_state < flightStates::kLanded)
//          SendTelemetryData();
      if (rocket_service_count_ == SAMPLES_PER_SECOND){ // Lower frequency conserves battery.
        if (flight_stats_.flight_state == FlightStates::kWaitingLaunch){ // Blink LoRa transmit LED for visual validation until liftoff
          HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
          if (HAL_GPIO_ReadPin(EMATCH_SENSE_1_GPIO_Port, EMATCH_SENSE_1_Pin))
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
          if (HAL_GPIO_ReadPin(EMATCH_SENSE_2_GPIO_Port, EMATCH_SENSE_2_Pin))
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
        }
        rocket_service_count_ = 0;
        SendTelemetryData();
      }
      flight_manager_.IncrementFlightDataQueue();
      break;
    case DeviceState::kConfig:
      break;
    case DeviceState::kConfigSavePending:
      SetDisplayDeployMode();
      rocket_file_.SaveRocketSettings(&rocket_settings_);
      Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
      device_state_ = DeviceState::kStandby;
      ResetDisplayDeployMode();
      break;
    case DeviceState::kTest:
      rocket_config_.ProcessTestDeploy();
      break;
  }
}

void RocketFactory::SendTelemetryData(){
  if (rocket_gps_.GPSDataValid()){
    rocket_gps_.SetFlightState(flight_stats_.flight_state, flight_stats_.sample_count);
    uint8_t telemetry_data_size = rocket_gps_.TelemetryDataSize();
    uint8_t agl_data_size;
    if (flight_stats_.flight_state > FlightStates::kWaitingLaunch && flight_stats_.flight_state < FlightStates::kDroguePrimaryDeployed)
      agl_data_size = SAMPLES_PER_SECOND;
    else
      agl_data_size = 1;
    uint8_t packet_size = telemetry_data_size + agl_data_size * sizeof(float);
    uint8_t telemetryPacket[packet_size] = {0};
    rocket_gps_.GgaToPacket(telemetryPacket);
    flight_manager_.AglToPacket(telemetryPacket + telemetry_data_size, agl_data_size);
    Radio.Send(telemetryPacket, packet_size);
  }
  else{
    const char* bad_gps_data = "Bad GPS Data\r\n";
    Radio.Send((uint8_t*)bad_gps_data, 14);
  }
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
