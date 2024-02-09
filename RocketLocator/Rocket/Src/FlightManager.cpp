#include "FlightManager.hpp"

FlightManager::FlightManager(){
}

FlightManager::FlightManager(RocketSettings *rocket_settings, SensorValues *sensor_values, FlightStats *flight_stats){
  rocket_settings_ = rocket_settings;
  sensor_values_ = sensor_values;
  flight_stats_ = flight_stats;
  SumSquares();
}

void FlightManager::Begin(){
  //APP_LOG(TS_OFF, VLEVEL_M, "\r\nFlightManager begin\r\n");
  MX_I2C2_Init();
  if (!accelerometer_.MC3416Init())
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  else{
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    APP_LOG(TS_OFF, VLEVEL_M, "\r\nAccelerometer initialization failed\r\n");
  }
  HAL_Delay(1000); //Allow time for init status light to stay visible
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  float x_axis = 0.0, y_axis = 0.0, z_axis = 0.0;
  Accelerometer_t accelerometer;
  accelerometer_.UpdateAccelerometerValues(&accelerometer);
  x_axis = accelerometer.x * accelerometer_.GetGRangeScale();
  y_axis = accelerometer.y * accelerometer_.GetGRangeScale();
  z_axis = accelerometer.z * accelerometer_.GetGRangeScale();
  g_force_short_sample_ = sqrt(x_axis * x_axis + y_axis * y_axis + z_axis * z_axis);
  Bmp280InitDefaultParams(&bmp280_.params);
  bmp280_.addr = BMP280_I2C_ADDRESS_0;
  bmp280_.i2c = &hi2c2;
  if (Bmp280Init(&bmp280_, &bmp280_.params))
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
  else{
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    APP_LOG(TS_OFF, VLEVEL_M, "\r\nBMP280 initialization failed\r\n");
  }
  HAL_Delay(1000); //Allow time for BME to stabilize, init status light to stay visible
  GetAGL();
  flight_stats_->agl_adjust = sensor_altitude_;
  for (int i = 0; i < FLIGHT_DATA_ARRAY_SIZE - 1; i++){ // Populate flight data array
    GetAltimeterData();
    GetAccelerometerData();
    flight_stats_->flight_data_array_index++;
    test_data_sample_count_++;
  }
  flight_stats_->sample_count = LAUNCH_LOOKBACK_SAMPLES;
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void FlightManager::FlightService(){
  GetAltimeterData();
  GetAccelerometerData();
  UpdateFlightState();
}

void FlightManager::IncrementFlightDataQueue(){
  if (flight_stats_->flight_state < flightStates::kLanded){
    test_data_sample_count_++;
    if (flight_stats_->flight_state > flightStates::kWaitingLaunch)
      flight_stats_->sample_count++;
    for (int i = 0; i < FLIGHT_DATA_ARRAY_SIZE - 1; i++){
      flight_stats_->agl[i] = flight_stats_->agl[i + 1];
      flight_stats_->accelerometer[i].x = flight_stats_->accelerometer[i + 1].x;
      flight_stats_->accelerometer[i].y = flight_stats_->accelerometer[i + 1].y;
      flight_stats_->accelerometer[i].z = flight_stats_->accelerometer[i + 1].z;
    }
  }
  //serviceBeeper();
}

void FlightManager::GetAltimeterData(){
  GetAGL();
  if (TEST)
    sensor_agl_ = test_agl_[test_data_sample_count_];
  flight_stats_->agl[flight_stats_->flight_data_array_index] = sensor_agl_; // get altitude above ground level
  UpdateVelocity();
  agl_adjust_count_++;
  if (flight_stats_->flight_state == flightStates::kWaitingLaunch && agl_adjust_count_ >= 60 * SAMPLES_PER_SECOND
      && velocity_short_sample_ <= 1.0 && velocity_long_sample_ <= 1.0){
    agl_adjust_count_ = 0;
    flight_stats_->agl_adjust = sensor_altitude_;
  }
  mAGL = sensor_agl_;
  mVelocityShortSample = velocity_short_sample_;
  mVelocityLongSample = velocity_long_sample_;
}

void FlightManager::GetAGL(){
    Bmp280ReadFloat(&bmp280_, &temperature_, &pressure_, &humidity_);
    sensor_altitude_ = (44330 * (1.0 - pow(pressure_ / 101325, 0.1903)));
  sensor_agl_ = sensor_altitude_ - flight_stats_->agl_adjust;
  if (flight_stats_->flight_state > flightStates::kWaitingLaunch)
    if (sensor_agl_ > flight_stats_->agl[flight_stats_->flight_data_array_index - 1] + MAX_ALTITUDE_SAMPLE_CHANGE || sensor_agl_ < flight_stats_->agl[flight_stats_->flight_data_array_index - 1] - MAX_ALTITUDE_SAMPLE_CHANGE)
      sensor_agl_ = flight_stats_->agl[flight_stats_->flight_data_array_index - 1]; // Compensate for spurious values
  //Serial.printf("%4.1f ", sensor_agl_);
}

void FlightManager::GetAccelerometerData(){
  Accelerometer_t accelerometer;
  accelerometer_.UpdateAccelerometerValues(&accelerometer);
  if (TEST){
    if (flight_stats_->flight_state == flightStates::kWaitingLaunch && flight_stats_->sample_count > 5)
      accelerometer.x = 4300;
    else if (flight_stats_->flight_state >= flightStates::kLaunched && flight_stats_->sample_count > 60)
      accelerometer.x = 100;
    else if (flight_stats_->flight_state >= flightStates::kDroguePrimaryDeployed)
      accelerometer.x = 2048;
  }
  flight_stats_->accelerometer[flight_stats_->flight_data_array_index].x = accelerometer.x;
  flight_stats_->accelerometer[flight_stats_->flight_data_array_index].y = accelerometer.y;
  flight_stats_->accelerometer[flight_stats_->flight_data_array_index].z = accelerometer.z;
  uint8_t i = 0;
  float accel_x = 0.0;
  float accel_y = 0.0;
  float accel_z = 0.0;
  float g_force = 0.0;
  g_force_short_sample_ = 0;
  g_force_long_sample_ = 0;
  for (int history_index = FLIGHT_DATA_ARRAY_SIZE - G_FORCE_SAMPLES_LONG; i < G_FORCE_SAMPLES_LONG; i++, history_index++){
    accel_x = flight_stats_->accelerometer[history_index].x * accelerometer_.GetGRangeScale();
    accel_y = flight_stats_->accelerometer[history_index].y * accelerometer_.GetGRangeScale();
    accel_z = flight_stats_->accelerometer[history_index].z * accelerometer_.GetGRangeScale();
    g_force = sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (i >= G_FORCE_SAMPLES_LONG - G_FORCE_SAMPLES_SHORT)
      g_force_short_sample_ += g_force;
    g_force_long_sample_ += g_force;
  }
  m_g_force = g_force;
  g_force_short_sample_ /= G_FORCE_SAMPLES_SHORT;
  g_force_long_sample_ /= G_FORCE_SAMPLES_LONG;
  mX = accelerometer.x;
  mY = accelerometer.y;
  mZ = accelerometer.z;
}

void FlightManager::UpdateFlightState(){ // Update flight state
  if (flight_stats_->flight_state == flightStates::kWaitingLaunch && flight_stats_->agl[flight_stats_->flight_data_array_index] > rocket_settings_->launch_detect_altitude // Detect launch
      && velocity_short_sample_ > LAUNCH_VELOCITY && g_force_short_sample_ > LAUNCH_G_FORCE){ // Minimum altitude, velocity, acceleration
    flight_stats_->flight_state = flightStates::kLaunched;
    flight_stats_->launch_detect_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
    flight_stats_->launch_detect_sample_count = flight_stats_->sample_count;
    flight_stats_->g_range_scale = accelerometer_.GetGRangeScale();
    //APP_LOG(TS_OFF, VLEVEL_M, "\r\nLaunch: %d %3.2f %3.2f %3.2f\r\n", flight_stats_->aglIndex, flight_stats_->agl[flight_stats_->aglIndex], velocityShortSample, velocityLongSample);
  }

  if (flight_stats_->flight_state >= flightStates::kLaunched && flight_stats_->flight_state < flightStates::kNoseover){
    updateMaxAltitude();
    if (g_force_short_sample_ < 0.5)
      if (flight_stats_->burnout_sample_count == 0){ // Detect burnout
        flight_stats_->flight_state = flightStates::kBurnout;
        flight_stats_->burnout_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
        flight_stats_->burnout_sample_count = flight_stats_->sample_count;
      }
    float sum1 = 0.0, sum2 = 0.0; // Detect nose over
    for (int i = flight_stats_->flight_data_array_index - SAMPLES_PER_SECOND + 1; i < flight_stats_->flight_data_array_index - SAMPLES_PER_SECOND / 2 + 1; i++){
      sum1 += flight_stats_->agl[i];
      sum2 += flight_stats_->agl[i + SAMPLES_PER_SECOND / 2];
    }
    if (sum2 <= sum1){
      flight_stats_->nose_over_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->nose_over_sample_count = flight_stats_->sample_count;
      flight_stats_->flight_state = flightStates::kNoseover;
      //APP_LOG(TS_OFF, VLEVEL_M, "\r\nNoseover: %d %3.2f %3.2f %3.2f\r\n", flight_stats_->aglIndex, flight_stats_->agl[flight_stats_->aglIndex], velocityShortSample, velocityLongSample);
    }
  }

  if (flight_stats_->flight_state >= flightStates::kNoseover){
    if (flight_stats_->flight_state < flightStates::kDroguePrimaryDeployed
        && flight_stats_->sample_count >= flight_stats_->nose_over_sample_count + SAMPLES_PER_SECOND * rocket_settings_->drogue_primary_deploy_delay / 10){ // Deploy drogue primary
      if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup || rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary)
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      flight_stats_->drogue_primary_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->drogue_primary_deploy_sample_count = flight_stats_->sample_count;
      flight_stats_->flight_state = flightStates::kDroguePrimaryDeployed;
    }

    if (flight_stats_->flight_state < flightStates::kDrogueBackupDeployed
        && flight_stats_->sample_count >= flight_stats_->nose_over_sample_count + SAMPLES_PER_SECOND * rocket_settings_->drogue_backup_deploy_delay / 10){ // Deploy drogue backup
      if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup)
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      else if (rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup)
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      flight_stats_->drogue_backup_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->drogue_backup_deploy_sample_count = flight_stats_->sample_count;
      flight_stats_->flight_state = flightStates::kDrogueBackupDeployed;
    }

    if (flight_stats_->flight_state < flightStates::kMainPrimaryDeployed
        && flight_stats_->agl[flight_stats_->flight_data_array_index] <= rocket_settings_->main_primary_deploy_altitude){ // Deploy main primary
      if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup)
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      else if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary)
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      flight_stats_->flight_state = flightStates::kMainPrimaryDeployed;
      flight_stats_->main_primary_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->main_primary_deploy_sample_count = flight_stats_->sample_count;
    }

    if (flight_stats_->flight_state < flightStates::kMainBackupDeployed
        && flight_stats_->agl[flight_stats_->flight_data_array_index] <= rocket_settings_->main_backup_deploy_altitude){ // Deploy main backup
      if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup || rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup)
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      flight_stats_->flight_state = flightStates::kMainBackupDeployed;
      flight_stats_->main_backup_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->main_backup_deploy_sample_count = flight_stats_->sample_count;
    }
  }

  if (HAL_GPIO_ReadPin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin) == GPIO_PIN_SET){
    if ((rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup || rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary)
        && flight_stats_->sample_count >= flight_stats_->drogue_primary_deploy_sample_count + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop drogue primary deployment signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kDroguePrimaryDeployed)
        flight_stats_->flight_state = flightStates::kDroguePrimarySignalOff;
    }
    if (rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup
        && flight_stats_->sample_count >= flight_stats_->drogue_backup_deploy_sample_count + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop drogue backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kDrogueBackupDeployed)
        flight_stats_->flight_state = flightStates::kDrogueBackupSignalOff;
    }
    if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup
        && flight_stats_->sample_count >= flight_stats_->main_primary_deploy_sample_count + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main primary deployment signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kMainPrimaryDeployed)
        flight_stats_->flight_state = flightStates::kMainPrimarySignalOff;
    }
  }

  if (HAL_GPIO_ReadPin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin) == GPIO_PIN_SET){
    if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup
        && flight_stats_->sample_count >= flight_stats_->drogue_backup_deploy_sample_count + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kDrogueBackupDeployed)
        flight_stats_->flight_state = flightStates::kDrogueBackupSignalOff;
    }
    if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary
        && flight_stats_->sample_count >= flight_stats_->main_primary_deploy_sample_count + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kMainPrimaryDeployed)
        flight_stats_->flight_state = flightStates::kMainPrimarySignalOff;
    }
    if ((rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup || rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup)
        && flight_stats_->sample_count >= flight_stats_->main_backup_deploy_sample_count + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kMainBackupDeployed)
        flight_stats_->flight_state = flightStates::kMainBackupSignalOff;
    }
  }

  if (flight_stats_->flight_state >= flightStates::kNoseover && flight_stats_->flight_state < flightStates::kLanded){ // Landing
    if (abs(velocity_long_sample_) < DESCENT_RATE_THRESHOLD && flight_stats_->agl[flight_stats_->flight_data_array_index] < MAX_LANDING_ALTITUDE){
      flight_stats_->flight_state = flightStates::kLanded;
      flight_stats_->landing_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->landing_sample_count = flight_stats_->sample_count;
    }
  }
  mFlightState = flight_stats_->flight_state;
}

void FlightManager::UpdateVelocity(){
  if (flight_stats_->flight_state < flightStates::kLanded && flight_stats_->flight_data_array_index >= VELOCITY_SAMPLES_LONG - 1){
    float sampleTimeShort = (1.0 - VELOCITY_SAMPLES_SHORT) / (SAMPLES_PER_SECOND * 2);
  	velocity_short_sample_ = 0.0;
    for (int i = flight_stats_->flight_data_array_index - VELOCITY_SAMPLES_SHORT + 1; i <= flight_stats_->flight_data_array_index; i++){
      velocity_short_sample_ += sampleTimeShort * flight_stats_->agl[i];
      sampleTimeShort += 1.0 / SAMPLES_PER_SECOND;
    }
    velocity_short_sample_ /= velocity_short_sum_sq_; //sum of sampleTimes ^ 2
    velocity_long_sample_ = 0.0;
    float sampleTimeLong = (1.0 - VELOCITY_SAMPLES_LONG) / (SAMPLES_PER_SECOND * 2);
    for (int i = flight_stats_->flight_data_array_index - VELOCITY_SAMPLES_LONG + 1; i <= flight_stats_->flight_data_array_index; i++){
      velocity_long_sample_ += sampleTimeLong * flight_stats_->agl[i];
      sampleTimeLong += 1.0 / SAMPLES_PER_SECOND;
    }
    velocity_long_sample_ /= velocity_long_sum_sq_; //sum of sampleTimes ^ 2
  }
}

void FlightManager::updateMaxAltitude(){
  if (flight_stats_->agl[flight_stats_->flight_data_array_index] > flight_stats_->max_altitude){ // Update max altitude for apogee determination
    flight_stats_->max_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
    flight_stats_->max_altitude_sample_count = flight_stats_->sample_count;
  }
}

void FlightManager::SumSquares(){
  float sampleTimeShort = (1.0 - VELOCITY_SAMPLES_SHORT) / (SAMPLES_PER_SECOND * 2);
  for (int i = (1 - VELOCITY_SAMPLES_SHORT) / 2; i <= (VELOCITY_SAMPLES_SHORT - 1) / 2; i++){
    velocity_short_sum_sq_ += sampleTimeShort * sampleTimeShort;
    //Serial.printf("velocity_short_sum_sq_: %3.2f %3.2f\n", sampleTimeShort, velocity_short_sum_sq_);
    sampleTimeShort += 1.0 / SAMPLES_PER_SECOND;
  }
  float sampleTimeLong = (1.0 - VELOCITY_SAMPLES_LONG) / (SAMPLES_PER_SECOND * 2);
  for (int i = (1 - VELOCITY_SAMPLES_LONG) / 2; i <= (VELOCITY_SAMPLES_LONG - 1) / 2; i++){
    velocity_long_sum_sq_ += sampleTimeLong * sampleTimeLong;
    //Serial.printf("velocity_long_sum_sq_: %3.2f %3.2f\n", sampleTimeLong, velocityLongSumSq);
    sampleTimeLong += 1.0 / SAMPLES_PER_SECOND;
  }
}

void FlightManager::serviceBeeper(){
  //if ((flightStats.armed && flightStats.flightState == flightStates::WAITING_LDA || flightStats.flightState == flightStates::LANDED) && peripheralInterruptCount == 0)
  //  speaker.beep(1, 200);
}

void FlightManager::AglToPacket(uint8_t *packet, uint8_t length){
  if (flight_stats_->flight_data_array_index >= length)
      memcpy(packet, &flight_stats_->agl[flight_stats_->flight_data_array_index - length + 1], sizeof(float) * length);
}

float FlightManager::GetGForceShortSample(){
  return g_force_short_sample_;
}
