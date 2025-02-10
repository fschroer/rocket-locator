#include <RocketFactory.hpp>

RocketFactory::RocketFactory() {
  //rocket_file_.SaveRocketSettings(&rocket_settings_); //for testing only
  rocket_file_.ReadRocketSettings(&rocket_settings_);
  if (rocket_settings_.archive_position > ARCHIVE_POSITIONS - 1)
    rocket_settings_.archive_position = 0;
  if (rocket_settings_.deploy_mode > kMaxDeployModeValue - 1)
    rocket_settings_.deploy_mode = DEFAULT_DEPLOY_MODE;
  if (rocket_settings_.launch_detect_altitude > MAX_LAUNCH_DETECT_ALTITUDE)
    rocket_settings_.launch_detect_altitude = DEFAULT_LAUNCH_DETECT_ALTITUDE;
  if (rocket_settings_.drogue_primary_deploy_delay > MAX_DROGUE_PRIMARY_DEPLOY_DELAY)
    rocket_settings_.drogue_primary_deploy_delay = DEFAULT_DROGUE_PRIMARY_DEPLOY_DELAY;
  if (rocket_settings_.drogue_backup_deploy_delay > MAX_DROGUE_BACKUP_DEPLOY_DELAY)
    rocket_settings_.drogue_backup_deploy_delay = DEFAULT_DROGUE_BACKUP_DEPLOY_DELAY;
  if (rocket_settings_.main_primary_deploy_altitude > MAX_MAIN_PRIMARY_DEPLOY_ALTITUDE)
    rocket_settings_.main_primary_deploy_altitude = DEFAULT_MAIN_PRIMARY_DEPLOY_ALTITUDE;
  if (rocket_settings_.main_backup_deploy_altitude > MAX_MAIN_BACKUP_DEPLOY_ALTITUDE)
    rocket_settings_.main_backup_deploy_altitude = DEFAULT_MAIN_BACKUP_DEPLOY_ALTITUDE;
  if (rocket_settings_.deploy_signal_duration > MAX_DEPLOY_SIGNAL_DURATION)
    rocket_settings_.deploy_signal_duration = DEFAULT_DEPLOY_SIGNAL_DURATION;
  if (rocket_settings_.lora_channel > MAX_LORA_CHANNEL)
  	rocket_settings_.lora_channel = DEFAULT_LORA_CHANNEL;
  flight_manager_ = FlightManager(&rocket_settings_, &flight_stats_);
  rocket_config_ = RocketConfig(&device_state_, &rocket_settings_);
}

void RocketFactory::Begin() {
  Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
  flight_manager_.Begin(&accelerometer_status_, &altimeter_init_status_);
  rocket_gps_.Begin();
  rocket_file_.OpenAltimeterArchiveWrite(rocket_settings_.archive_position);
  rocket_file_.OpenAccelerometerArchiveWrite(rocket_settings_.archive_position);
  //SetDisplayDeployMode();
  //HAL_Delay(1000);
  //ResetDisplayDeployMode();
  Radio.Send((uint8_t*)lora_startup_message_, strlen(lora_startup_message_));
}

void RocketFactory::ProcessRocketEvents(uint8_t rocket_service_count) {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
  if(HAL_GPIO_ReadPin(POWER_SENSE_GPIO_Port, POWER_SENSE_Pin))
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  else
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  switch (device_state_){
    case DeviceState::kStandby:
      if (rocket_service_count == SAMPLES_PER_SECOND - 10){
        battery_voltage_mvolt_ = GetBatteryLevel();
        TransmitLEDsOn();
        SendPreLaunchData();
      }
      break;
    case DeviceState::kRunning:
      flight_manager_.GetAccelerometerData();
      flight_manager_.GetAGL();
      flight_manager_.UpdateVelocity();
      flight_manager_.UpdateFlightState(rocket_file_);
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
        rocket_file_.WriteFlightStats((uint32_t)&flight_stats_, (uint32_t)&flight_stats_.launch_date);
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
      if (flight_stats_.flight_state == FlightStates::kLanded && !altimeter_archive_closed_){
        //rocket_file_.WriteFlightMetadata(&flight_stats_);
        rocket_file_.WriteAltimeterSample(flight_stats_.agl[flight_stats_.flight_data_array_index]);
        rocket_file_.CloseAltimeterArchive();
        rocket_file_.UpdateArchivePosition(&rocket_settings_);
        rocket_file_.SaveRocketSettings(&rocket_settings_);
        altimeter_archive_closed_ = true;
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
      //SetDisplayDeployMode();
      rocket_file_.SaveRocketSettings(&rocket_settings_);
      Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
      device_state_ = DeviceState::kStandby;
      //ResetDisplayDeployMode();
      break;
    case DeviceState::kTest:
      int16_t test_deploy_count = rocket_config_.GetTestDeployCount();
      if (test_deploy_count >= 0 && test_deploy_count % SAMPLES_PER_SECOND == 0){
        char test_countdown[4] = {'T', 'S', 'T'};
        test_countdown[3] = test_deploy_count / SAMPLES_PER_SECOND;
        Radio.Send((uint8_t*)test_countdown, 4);
      }
      rocket_config_.ProcessTestDeploy();
      break;
  }
}

void RocketFactory::SendPreLaunchData() {
    switch (flight_profile_state_) {
      case FlightProfileState::kIdle:
        if (rocket_gps_.GPSDataValid()){
          uint8_t* packet_ptr = pre_launch_msg_ + LORA_MSG_TYPE_SIZE;
          rocket_gps_.GPSToPacket(packet_ptr); // Add GPS data to pre-launch message
          packet_ptr += sizeof(GPSData);
          uint8_t device_status = device_state_ << 4; // Add locator device state to pre-launch message
          device_status |= altimeter_init_status_ << 3; // Add altimeter initialization status to pre-launch message
          flight_manager_.GetAccelerometerData(); // Get accelerometer data
          if (flight_stats_.accelerometer[flight_stats_.flight_data_array_index].x != 0
              || flight_stats_.accelerometer[flight_stats_.flight_data_array_index].y != 0
              || flight_stats_.accelerometer[flight_stats_.flight_data_array_index].z != 0){
            accelerometer_status_ = MC_OK;
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
          }
          else{
            accelerometer_status_ = MC_Rd_Error;
            HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
          }
          device_status |= (accelerometer_status_ == MC_OK ? 1 : 0) << 2; // Add accelerometer initialization status to pre-launch message
          device_status |= HAL_GPIO_ReadPin(EMATCH_SENSE_1_GPIO_Port, EMATCH_SENSE_1_Pin) << 1;
          device_status |= HAL_GPIO_ReadPin(EMATCH_SENSE_2_GPIO_Port, EMATCH_SENSE_2_Pin);
          *packet_ptr = device_status;
          packet_ptr += sizeof(device_status);
          flight_manager_.GetAGL(); // Add altimeter data to pre-launch message
          flight_manager_.AglToPacket(packet_ptr, 1);
          packet_ptr += sizeof(uint16_t);
          *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].x; // Add accelerometer x to pre-launch message
          packet_ptr += sizeof((*flight_stats_.accelerometer).x);
          *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].y; // Add accelerometer y to pre-launch message
          packet_ptr += sizeof((*flight_stats_.accelerometer).y);
          *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].z; // Add accelerometer z to pre-launch message
          packet_ptr += sizeof((*flight_stats_.accelerometer).z);
          *(DeployMode*)packet_ptr = rocket_settings_.deploy_mode;
          packet_ptr += sizeof(rocket_settings_.deploy_mode);
          *(int*)packet_ptr = rocket_settings_.launch_detect_altitude;
          packet_ptr += sizeof(rocket_settings_.launch_detect_altitude);
          *(int*)packet_ptr = rocket_settings_.drogue_primary_deploy_delay;
          packet_ptr += sizeof(rocket_settings_.drogue_primary_deploy_delay);
          *(int*)packet_ptr = rocket_settings_.drogue_backup_deploy_delay;
          packet_ptr += sizeof(rocket_settings_.drogue_backup_deploy_delay);
          *(int*)packet_ptr = rocket_settings_.main_primary_deploy_altitude;
          packet_ptr += sizeof(rocket_settings_.main_primary_deploy_altitude);
          *(int*)packet_ptr = rocket_settings_.main_backup_deploy_altitude;
          packet_ptr += sizeof(rocket_settings_.main_backup_deploy_altitude);
          *(int*)packet_ptr = rocket_settings_.deploy_signal_duration;
          packet_ptr += sizeof(rocket_settings_.deploy_signal_duration);
          for (int i = 0; i < DEVICE_NAME_LENGTH + 1; i++)
            *(char*)(packet_ptr++) = rocket_settings_.device_name[i];
          *(uint16_t*)packet_ptr = battery_voltage_mvolt_;

          Radio.Send(pre_launch_msg_, sizeof(pre_launch_msg_));
        }
        else{
          m_bad_gps_message++;
          Radio.Send((uint8_t*)bad_gps_data_, strlen(bad_gps_data_));
        }
        break;
      case FlightProfileState::kMetadataRequested:
        SendFlightProfileMetadata();
        flight_profile_state_ = FlightProfileState::kIdle;
        break;
      case FlightProfileState::kDataRequested:
        SendFlightProfileData(flight_profile_archive_position_);
        flight_profile_state_ = FlightProfileState::kIdle;
        break;
    }
  m_radio_send = !m_radio_send;
}

void RocketFactory::SendTelemetryData() {
  if (rocket_gps_.GPSDataValid()){
    uint8_t* packet_ptr = telemetry_msg_ + LORA_MSG_TYPE_SIZE;
    rocket_gps_.GPSToPacket(packet_ptr); // Add GPS data
    packet_ptr += sizeof(GPSData);
    uint8_t device_status = device_state_ << 4; // Add locator device state
    device_status |= altimeter_init_status_ << 3; // Add altimeter initialization status
    flight_manager_.GetAccelerometerData(); // Get accelerometer data
    if (flight_stats_.accelerometer[flight_stats_.flight_data_array_index].x != 0
        || flight_stats_.accelerometer[flight_stats_.flight_data_array_index].y != 0
        || flight_stats_.accelerometer[flight_stats_.flight_data_array_index].z != 0){
      accelerometer_status_ = MC_OK;
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    else{
      accelerometer_status_ = MC_Rd_Error;
      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    }
    device_status |= (accelerometer_status_ == MC_OK ? 1 : 0) << 2; // Add accelerometer initialization status
    device_status |= HAL_GPIO_ReadPin(EMATCH_SENSE_1_GPIO_Port, EMATCH_SENSE_1_Pin) << 1;
    device_status |= HAL_GPIO_ReadPin(EMATCH_SENSE_2_GPIO_Port, EMATCH_SENSE_2_Pin);
    *packet_ptr = device_status;
    packet_ptr += sizeof(device_status);
    flight_manager_.GetAGL(); // Add altimeter data
    flight_manager_.AglToPacket(packet_ptr, 1);
    packet_ptr += sizeof(uint16_t);
    *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].x; // Add accelerometer x
    packet_ptr += sizeof((*flight_stats_.accelerometer).x);
    *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].y; // Add accelerometer y
    packet_ptr += sizeof((*flight_stats_.accelerometer).y);
    *(uint16_t*)packet_ptr = flight_stats_.accelerometer[flight_stats_.flight_data_array_index].z; // Add accelerometer z
    packet_ptr += sizeof((*flight_stats_.accelerometer).z);
    *(float*)packet_ptr = flight_manager_.GetVelocityShortSample();
    packet_ptr += sizeof(float);
    *packet_ptr = flight_stats_.flight_state; // Add flight state
    packet_ptr += sizeof(flight_stats_.flight_state);
    *(uint16_t*)packet_ptr = flight_stats_.sample_count; // Add sample count
    packet_ptr += sizeof(flight_stats_.sample_count);
    //uint8_t agl_data_size = 1;  // Add altimeter data
    //if (flight_stats_.flight_state > FlightStates::kWaitingLaunch && flight_stats_.flight_state < FlightStates::kDroguePrimaryDeployed)
    //  agl_data_size = SAMPLES_PER_SECOND;
    //flight_manager_.AglToPacket(packet_ptr, agl_data_size);
    //Radio.Send(telemetry_msg_, sizeof(telemetry_msg_) - (SAMPLES_PER_SECOND - agl_data_size) * sizeof(uint16_t));
    Radio.Send(telemetry_msg_, sizeof(telemetry_msg_));
  }
  else{
    m_bad_gps_message++;
    Radio.Send((uint8_t*)bad_gps_data_, strlen(bad_gps_data_));
  }
  m_radio_send = !m_radio_send;
}

void RocketFactory::SendFlightProfileMetadata() {
  FlightStats flight_stats;
  uint8_t* packet_ptr = flight_profile_metadata_msg_ + LORA_MSG_TYPE_SIZE;
  for (int i = 0; i < ARCHIVE_POSITIONS; i++){
    rocket_file_.ReadFlightMetadata(i, &flight_stats);
    if (flight_stats.max_altitude > flight_stats.launch_detect_altitude && flight_stats.max_altitude > flight_stats.landing_altitude
        && flight_stats.landing_sample_count > flight_stats.launch_detect_sample_count){
      *(int*)packet_ptr = flight_stats.launch_date;
      packet_ptr += sizeof(flight_stats.launch_date);
      *(int*)packet_ptr = flight_stats.launch_time;
      packet_ptr += sizeof(flight_stats.launch_time);
      *(float*)packet_ptr = flight_stats.max_altitude;
      packet_ptr += sizeof(flight_stats.max_altitude);
      *(float*)packet_ptr = (float)flight_stats.max_altitude_sample_count / SAMPLES_PER_SECOND;
      packet_ptr += sizeof(float);
    }
    else {
      *(int*)packet_ptr = 0;
      packet_ptr += sizeof(flight_stats.launch_date);
      *(int*)packet_ptr = 0;
      packet_ptr += sizeof(flight_stats.launch_time);
      *(float*)packet_ptr = 0;
      packet_ptr += sizeof(flight_stats.max_altitude);
      *(float*)packet_ptr = (float)0;
      packet_ptr += sizeof(float);
    }
  }
  //Radio.Send((uint8_t*)lora_startup_message_, strlen(lora_startup_message_));
  Radio.Send(flight_profile_metadata_msg_, sizeof(flight_profile_metadata_msg_));
}

void RocketFactory::SendFlightProfileData(uint8_t archive_position) {
  FlightStats flight_data_stats;
  rocket_file_.ReadFlightMetadata(archive_position, &flight_data_stats);

  int packet_index = 0;
  int sample_index = 0;
  uint16_t agl = 0;
  Accelerometer_t accelerometer;
  bool accelerometer_data_present = true;

  uint8_t* packet_ptr = flight_profile_data_msg_ + LORA_MSG_TYPE_SIZE;
  HAL_Delay(60); // Pause to ensure receiver has finished processing other packets (total first pause = 60 + 40 = 100ms)
  while (rocket_file_.ReadAltimeterData(archive_position, sample_index, flight_data_stats.landing_sample_count, &agl)){
    rocket_file_.ReadAccelerometerData(archive_position, sample_index
        , flight_data_stats.drogue_primary_deploy_sample_count, &accelerometer);
    accelerometer_data_present = (accelerometer.x != -1 || accelerometer.y != -1 || accelerometer.z != -1);
    if (packet_ptr == flight_profile_data_msg_ + LORA_MSG_TYPE_SIZE) {
      *packet_ptr = packet_index++; // packet sequence
      packet_ptr++;
    }
    *(uint16_t*)packet_ptr = agl;
    packet_ptr += sizeof(uint16_t);
    if (accelerometer_data_present) {
      *(Accelerometer_t*)packet_ptr = accelerometer;
      packet_ptr += sizeof(accelerometer);
    }
    sample_index++;
    if (packet_ptr - flight_profile_data_msg_ - LORA_MSG_TYPE_SIZE - FLIGHT_DATA_SEQUENCE_SIZE == FLIGHT_DATA_MESSAGE_SIZE) {
      while (!ready_to_send_) HAL_Delay(10);
      HAL_Delay(5); // Pause to ensure receiver processes prior packets
      Radio.Send(flight_profile_data_msg_, sizeof(flight_profile_data_msg_));
      ready_to_send_ = false;
      packet_ptr = flight_profile_data_msg_ + LORA_MSG_TYPE_SIZE;
      sample_index = 0;
    }
  }
}

void RocketFactory::TransmitLEDsOn() {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  if (HAL_GPIO_ReadPin(EMATCH_SENSE_1_GPIO_Port, EMATCH_SENSE_1_Pin))
    HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
  if (HAL_GPIO_ReadPin(EMATCH_SENSE_2_GPIO_Port, EMATCH_SENSE_2_Pin))
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
}

void RocketFactory::ProcessIncomingLoRaMessage(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo) {
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  //GPIO_PinState usb_connected = HAL_GPIO_ReadPin(POWER_SENSE_GPIO_Port, POWER_SENSE_Pin);
  if (payload[0] == 'R' && payload[1] == 'u' && payload[2] == 'n' && payload[3] == '\r'){
    //if (usb_connected != GPIO_PIN_SET)
      device_state_ = kRunning;
    //else
    //  Radio.Send((uint8_t*)usb_connected_, strlen(usb_connected_));
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Run", 3, UART_TIMEOUT);
  }
  else if (payload[0] == 'S' && payload[1] == 't' && payload[2] == 'o' && payload[3] == 'p' && payload[4] == '\r') {
    device_state_ = kStandby;
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Stop", 3, UART_TIMEOUT);
  }
  else if (payload[0] == 'C' && payload[1] == 'F' && payload[2] == 'G') {
    rocket_settings_.deploy_mode = (DeployMode)payload[3];
    rocket_settings_.launch_detect_altitude = *(uint16_t*)(payload + 4);
    rocket_settings_.drogue_primary_deploy_delay = (uint8_t)payload[6];
    rocket_settings_.drogue_backup_deploy_delay = (uint8_t)payload[7];
    rocket_settings_.main_primary_deploy_altitude = *(uint16_t*)(payload + 8);
    rocket_settings_.main_backup_deploy_altitude = *(uint16_t*)(payload + 10);
    rocket_settings_.deploy_signal_duration = (uint8_t)payload[12];
    for (int i = 0; i < DEVICE_NAME_LENGTH + 1; i++)
      rocket_settings_.device_name[i] = (char)payload[i + 13];
    rocket_file_.SaveRocketSettings(&rocket_settings_);
  }
  else if (payload[0] == 'F' && payload[1] == 'P' && payload[2] == 'M') {
    flight_profile_state_ = FlightProfileState::kMetadataRequested;
  }
  else if (payload[0] == 'F' && payload[1] == 'P' && payload[2] == 'D') {
    flight_profile_state_ = FlightProfileState::kDataRequested;
    flight_profile_archive_position_ = payload[3];
    if (flight_profile_archive_position_ < 0 || flight_profile_archive_position_ > ARCHIVE_POSITIONS)
      flight_profile_archive_position_ = 0;
  }
  else if (payload[0] == 'T' && payload[1] == 'S' && payload[2] == 'T'){
    switch ((UserInteractionState)payload[3]){
      case UserInteractionState::kTestDeploy1:
        rocket_config_.ResetTestDeployCount();
        device_state_ = DeviceState::kTest;
        rocket_config_.SetUserInteractionState(UserInteractionState::kTestDeploy1);
        break;
      case UserInteractionState::kTestDeploy2:
        rocket_config_.ResetTestDeployCount();
        device_state_ = DeviceState::kTest;
        rocket_config_.SetUserInteractionState(UserInteractionState::kTestDeploy2);
        break;
      case UserInteractionState::kWaitingForCommand:
        device_state_ = DeviceState::kStandby;
        rocket_config_.SetUserInteractionState(UserInteractionState::kWaitingForCommand);
        break;
      default:
        break;
      }
  }
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void RocketFactory::ProcessUART1Char(uint8_t uart_char) {
  rocket_gps_.ProcessChar(uart_char);
}

void RocketFactory::ProcessUART2Char(UART_HandleTypeDef *huart2, uint8_t uart_char) {
  rocket_config_.ProcessChar(uart_char);
}

uint16_t RocketFactory::GetBatteryLevel() {
#ifndef BOARD_REVISION_A_04
  /* Init variable containing ADC conversion data */
  uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE;

  /* Perform ADC group regular conversion start, poll for conversion        */
  /* completion.                                                            */
  ConversionStartPoll_ADC_GrpRegular();

  /* Retrieve ADC conversion data */
  /* (data scale corresponds to ADC resolution: 12 bits) */
  uhADCxConvertedData = LL_ADC_REG_ReadConversionData12(ADC);

  /* Computation of ADC conversions raw data to physical values           */
  /* using LL ADC driver helper macro.                                    */
  uhADCxConvertedData_Voltage_mVolt = __LL_ADC_CALC_DATA_TO_VOLTAGE(VDDA_APPLI, uhADCxConvertedData, LL_ADC_RESOLUTION_12B);
#endif
  return uhADCxConvertedData_Voltage_mVolt * BATTERY_LEVEL_VOLTAGE_DIVIDER_ADJUST;
}

void RocketFactory::ReadyToSend() {
  ready_to_send_ = true;
}

/*
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
*/
