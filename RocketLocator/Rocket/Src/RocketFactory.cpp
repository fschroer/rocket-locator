#include <RocketFactory.hpp>

RocketFactory::RocketFactory(){
  //rocket_file_.SaveRocketSettings(&rocket_settings_); //for testing only
  rocket_file_.ReadRocketSettings(&rocket_settings_);
  if (rocket_settings_.lora_channel > MAX_LORA_CHANNEL)
  	rocket_settings_.lora_channel = 0;
  flight_manager_ = FlightManager(&rocket_settings_, &flight_stats_);
  rocket_config_ = RocketConfig(&device_state_, &rocket_settings_);
}

void RocketFactory::Begin(){
  Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
  flight_manager_.Begin(&accelerometer_init_status_, &altimeter_init_status_);
  rocket_gps_.Begin();
  rocket_file_.OpenAltimeterArchiveWrite(rocket_settings_.archive_position);
  rocket_file_.OpenAccelerometerArchiveWrite(rocket_settings_.archive_position);
  SetDisplayDeployMode();
  HAL_Delay(1000);
  ResetDisplayDeployMode();
  Radio.Send((uint8_t*)lora_startup_message_, strlen(lora_startup_message_));
}

void RocketFactory::ProcessRocketEvents(uint8_t rocket_service_count){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
  switch (device_state_){
    case DeviceState::kStandby:
      if (rocket_service_count == SAMPLES_PER_SECOND - 10){
        TransmitLEDsOn();
        SendPreLaunchData();
      }
      break;
    case DeviceState::kRunning:
      flight_manager_.GetAccelerometerData();
      flight_manager_.GetAGL();
      flight_manager_.UpdateVelocity();
      flight_manager_.UpdateFlightState();
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
        else if (rocket_service_count == 0){
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
//      if (rocket_service_count == SAMPLES_PER_SECOND / 2)
//        if (flight_stats_.flight_state > flightStates::kWaitingLaunch && flight_stats_.flight_state < flightStates::kLanded)
//          SendTelemetryData();
      if (rocket_service_count == SAMPLES_PER_SECOND - 10){ // Lower frequency conserves battery.
        if (flight_stats_.flight_state == FlightStates::kWaitingLaunch){ // Blink LoRa transmit LED for visual validation until liftoff
          TransmitLEDsOn();
        }
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

void RocketFactory::SendPreLaunchData(){
  if (rocket_gps_.GPSDataValid()){
    uint8_t* packet_ptr = pre_launch_msg_ + LORA_MSG_TYPE_SIZE;
    rocket_gps_.GPSToPacket(packet_ptr); // Add GPS data to pre-launch message
    packet_ptr += sizeof(GPSData);
    *packet_ptr = (uint8_t)altimeter_init_status_; // Add altimeter initialization status to pre-launch message
    packet_ptr += sizeof(uint8_t);
    flight_manager_.GetAGL(); // Add altimeter data to pre-launch message
    flight_manager_.AglToPacket(packet_ptr, 1);
    packet_ptr += sizeof(uint16_t);
    *packet_ptr = (uint8_t)accelerometer_init_status_; // Add accelerometer initialization status to pre-launch message
    packet_ptr += sizeof(uint8_t);
    flight_manager_.GetAccelerometerData(); // Add accelerometer data to pre-launch message
    *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].x;
    packet_ptr += sizeof((*flight_stats_.accelerometer).x);
    *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].y;
    packet_ptr += sizeof((*flight_stats_.accelerometer).y);
    *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].z;
    packet_ptr += sizeof((*flight_stats_.accelerometer).z);
    uint8_t deploy_sense = rocket_settings_.deploy_mode << 2; // Add deployment configuration and status to pre-launch message
    deploy_sense |= HAL_GPIO_ReadPin(EMATCH_SENSE_1_GPIO_Port, EMATCH_SENSE_1_Pin) << 1;
    deploy_sense |= HAL_GPIO_ReadPin(EMATCH_SENSE_2_GPIO_Port, EMATCH_SENSE_2_Pin);
    *packet_ptr = deploy_sense;
    packet_ptr += sizeof(deploy_sense);
    *(uint16_t*)packet_ptr = (uint16_t)rocket_settings_.launch_detect_altitude; // Add launch detect altitude to pre-launch message
    packet_ptr += sizeof(uint16_t);
    uint16_t deploy1_trigger_threshold, deploy2_trigger_threshold; // Add deployment trigger thresholds to pre-launch message
    switch (rocket_settings_.deploy_mode){
    case DeployMode::kDroguePrimaryDrogueBackup:
      deploy1_trigger_threshold = (uint16_t)rocket_settings_.drogue_primary_deploy_delay;
      deploy2_trigger_threshold = (uint16_t)rocket_settings_.drogue_backup_deploy_delay;
      break;
    case DeployMode::kMainPrimaryMainBackup:
      deploy1_trigger_threshold = (uint16_t)rocket_settings_.main_primary_deploy_altitude;
      deploy2_trigger_threshold = (uint16_t)rocket_settings_.main_backup_deploy_altitude;
      break;
    case DeployMode::kDroguePrimaryMainPrimary:
      deploy1_trigger_threshold = (uint16_t)rocket_settings_.drogue_primary_deploy_delay;
      deploy2_trigger_threshold = (uint16_t)rocket_settings_.main_primary_deploy_altitude;
      break;
    case DeployMode::kDrogueBackupMainBackup:
      deploy1_trigger_threshold = (uint16_t)rocket_settings_.drogue_backup_deploy_delay;
      deploy2_trigger_threshold = (uint16_t)rocket_settings_.main_backup_deploy_altitude;
      break;
    }
    *(uint16_t*)packet_ptr = deploy1_trigger_threshold;
    packet_ptr += sizeof(deploy1_trigger_threshold);
    *(uint16_t*)packet_ptr = deploy2_trigger_threshold;
    packet_ptr += sizeof(deploy2_trigger_threshold);
    *(uint16_t*)packet_ptr = (uint16_t)rocket_settings_.deploy_signal_duration; // Add deployment signal duration to pre-launch message

    Radio.Send(pre_launch_msg_, sizeof(pre_launch_msg_));
  }
  else{
    m_bad_gps_message++;
    Radio.Send((uint8_t*)bad_gps_data_, strlen(bad_gps_data_));
  }
  m_radio_send = !m_radio_send;
}

void RocketFactory::SendTelemetryData(){
  if (rocket_gps_.GPSDataValid()){
    uint8_t* packet_ptr = telemetry_msg_ + LORA_MSG_TYPE_SIZE;
    rocket_gps_.GPSToPacket(packet_ptr); // Add GPS data to pre-launch message
    packet_ptr += sizeof(GPSData);
    *packet_ptr = flight_stats_.flight_state; // Add flight state to pre-launch message
    packet_ptr += sizeof(flight_stats_.flight_state);
    *(uint16_t*)packet_ptr = flight_stats_.sample_count; // Add sample count to pre-launch message
    packet_ptr += sizeof(flight_stats_.sample_count);
    uint8_t agl_data_size = 1;  // Add altimeter data to pre-launch message
    if (flight_stats_.flight_state > FlightStates::kWaitingLaunch && flight_stats_.flight_state < FlightStates::kDroguePrimaryDeployed)
      agl_data_size = SAMPLES_PER_SECOND;
    flight_manager_.AglToPacket(packet_ptr, agl_data_size);
    Radio.Send(telemetry_msg_, sizeof(telemetry_msg_) - (SAMPLES_PER_SECOND - agl_data_size) * sizeof(uint16_t));
  }
  else{
    m_bad_gps_message++;
    Radio.Send((uint8_t*)bad_gps_data_, strlen(bad_gps_data_));
  }
  m_radio_send = !m_radio_send;
}

void RocketFactory::TransmitLEDsOn(){
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  if (HAL_GPIO_ReadPin(EMATCH_SENSE_1_GPIO_Port, EMATCH_SENSE_1_Pin))
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
  if (HAL_GPIO_ReadPin(EMATCH_SENSE_2_GPIO_Port, EMATCH_SENSE_2_Pin))
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
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
