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
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
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
  for (int i = 0; i < FLIGHT_DATA_ARRAY_SIZE - 1; i++){ // Populate flight data array
    GetAccelerometerData();
    GetAGL();
    flight_stats_->flight_data_array_index++;
    flight_stats_->test_data_sample_count++;
  }
  flight_stats_->sample_count = LAUNCH_LOOKBACK_SAMPLES;
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void FlightManager::FlightService(){
  GetAccelerometerData();
  GetAGL();
  UpdateVelocity();
  UpdateFlightState();
}

void FlightManager::IncrementFlightDataQueue(){
  if (flight_stats_->flight_state < FlightStates::kLanded){
    flight_stats_->test_data_sample_count++;
    for (int i = 0; i < FLIGHT_DATA_ARRAY_SIZE - 1; i++){
      flight_stats_->agl[i] = flight_stats_->agl[i + 1];
      flight_stats_->accelerometer[i].x = flight_stats_->accelerometer[i + 1].x;
      flight_stats_->accelerometer[i].y = flight_stats_->accelerometer[i + 1].y;
      flight_stats_->accelerometer[i].z = flight_stats_->accelerometer[i + 1].z;
    }
  }
  //serviceBeeper();
}

void FlightManager::GetAGL(){
  Bmp280ReadFloat(&bmp280_, &temperature_, &pressure_, &humidity_);
  sensor_altitude_ = (44330 * (1.0 - pow(pressure_ / 101325, 0.1903)));
  if (flight_stats_->flight_state == FlightStates::kWaitingLaunch && g_force_short_sample_ < MAX_AGL_ADJUST_G_FORCE){
    if (agl_adjust_count_ == 0){
      flight_stats_->agl_adjust = sensor_altitude_;
    }
    agl_adjust_count_++;
    if (agl_adjust_count_ > AGL_RESET_TIME)
      agl_adjust_count_ = 0;
  }
  flight_stats_->agl[flight_stats_->flight_data_array_index] = sensor_altitude_ - flight_stats_->agl_adjust;
//  if (TEST)
//    flight_stats_->agl[flight_stats_->flight_data_array_index] = test_agl_[flight_stats_->test_data_sample_count];
  mAGL = flight_stats_->agl[flight_stats_->flight_data_array_index];
}

void FlightManager::GetAccelerometerData(){
  Accelerometer_t accelerometer;
  accelerometer_.UpdateAccelerometerValues(&accelerometer);
/*  if (TEST){
    if (flight_stats_->flight_state == FlightStates::kWaitingLaunch && flight_stats_->sample_count > 5){
      accelerometer.x = 4300;
      accelerometer.y = 0;
      accelerometer.z = 0;
    }
    else if (flight_stats_->flight_state >= FlightStates::kLaunched && flight_stats_->sample_count > 60){
      accelerometer.x = 100;
      accelerometer.y = 0;
      accelerometer.z = 0;
    }
    else if (flight_stats_->flight_state >= FlightStates::kDroguePrimaryDeployed){
      accelerometer.x = 2048;
      accelerometer.y = 0;
      accelerometer.z = 0;
    }
  }*/
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
  g_force_last_ = g_force;
  g_force_short_sample_ /= G_FORCE_SAMPLES_SHORT;
  g_force_long_sample_ /= G_FORCE_SAMPLES_LONG;
  m_g_force = g_force;
  mX = accelerometer.x;
  mY = accelerometer.y;
  mZ = accelerometer.z;
}

void FlightManager::UpdateFlightState(){ // Update flight state
  if (flight_stats_->flight_state == FlightStates::kWaitingLaunch && flight_stats_->agl[flight_stats_->flight_data_array_index] > rocket_settings_->launch_detect_altitude // Detect launch
      && velocity_short_sample_ > LAUNCH_VELOCITY// && g_force_short_sample_ > LAUNCH_G_FORCE
      ){ // Minimum altitude, velocity, acceleration
    ResetFlightStats(true);
    flight_stats_->flight_state = FlightStates::kLaunched;
    flight_stats_->launch_detect_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
    flight_stats_->launch_detect_sample_count = flight_stats_->sample_count;
    flight_stats_->g_range_scale = accelerometer_.GetGRangeScale();
    //APP_LOG(TS_OFF, VLEVEL_M, "\r\nLaunch: %d %3.2f %3.2f %3.2f\r\n", flight_stats_->aglIndex, flight_stats_->agl[flight_stats_->aglIndex], velocityShortSample, velocityLongSample);
  }

  if (flight_stats_->flight_state >= FlightStates::kLaunched && flight_stats_->flight_state < FlightStates::kNoseover){
    updateMaxAltitude();
    if (g_force_short_sample_ > 3.0)
      accelerometer_state_ = AccelerometerStates::kAcceleration;
    else if (g_force_short_sample_ < 1.0)
      accelerometer_state_ = AccelerometerStates::kDeceleration;
    if (accelerometer_state_ != AccelerometerStates::kAcceleration){
      if (flight_stats_->burnout_sample_count == 0){ // Detect burnout
        flight_stats_->flight_state = FlightStates::kBurnout;
        flight_stats_->burnout_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
        flight_stats_->burnout_sample_count = flight_stats_->sample_count;
      }
      float sum1 = 0.0, sum2 = 0.0; // Detect nose over
      for (int i = flight_stats_->flight_data_array_index - SAMPLES_PER_SECOND + 1; i < flight_stats_->flight_data_array_index - SAMPLES_PER_SECOND / 2 + 1; i++){
        sum1 += flight_stats_->agl[i];
        sum2 += flight_stats_->agl[i + SAMPLES_PER_SECOND / 2];
      }
      if (sum2 <= sum1){
        noseover_time_ = 0;
        flight_stats_->nose_over_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
        flight_stats_->nose_over_sample_count = flight_stats_->sample_count;
        flight_stats_->flight_state = FlightStates::kNoseover;
        //APP_LOG(TS_OFF, VLEVEL_M, "\r\nNoseover: %d %3.2f %3.2f %3.2f\r\n", flight_stats_->aglIndex, flight_stats_->agl[flight_stats_->aglIndex], velocityShortSample, velocityLongSample);
      }
    }
  }

  if (flight_stats_->flight_state >= FlightStates::kNoseover){
    if (flight_stats_->flight_state < FlightStates::kDroguePrimaryDeployed
        && noseover_time_ >= SAMPLES_PER_SECOND * rocket_settings_->drogue_primary_deploy_delay / 10){ // Deploy drogue primary
      if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup || rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary){
        deploy_1_time_ = 0;
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      }
      flight_stats_->drogue_primary_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->drogue_primary_deploy_sample_count = flight_stats_->sample_count;
      flight_stats_->flight_state = FlightStates::kDroguePrimaryDeployed;
    }

    if (flight_stats_->flight_state < FlightStates::kDrogueBackupDeployed
        && noseover_time_ >= SAMPLES_PER_SECOND * rocket_settings_->drogue_backup_deploy_delay / 10){ // Deploy drogue backup
      if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup){
        deploy_2_time_ = 0;
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      }
      else if (rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup){
        deploy_1_time_ = 0;
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      }
      flight_stats_->drogue_backup_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->drogue_backup_deploy_sample_count = flight_stats_->sample_count;
      flight_stats_->flight_state = FlightStates::kDrogueBackupDeployed;
    }

    if (flight_stats_->flight_state < FlightStates::kMainPrimaryDeployed
        && (flight_stats_->agl[flight_stats_->flight_data_array_index] <= rocket_settings_->main_primary_deploy_altitude
            || velocity_short_sample_ > FREE_FALL_THRESHOLD)){ // Deploy main primary
      if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup){
        deploy_1_time_ = 0;
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      }
      else if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary){
        deploy_2_time_ = 0;
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      }
      flight_stats_->flight_state = FlightStates::kMainPrimaryDeployed;
      flight_stats_->main_primary_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->main_primary_deploy_sample_count = flight_stats_->sample_count;
    }

    if (flight_stats_->flight_state < FlightStates::kMainBackupDeployed
        && (flight_stats_->agl[flight_stats_->flight_data_array_index] <= rocket_settings_->main_backup_deploy_altitude
            || velocity_short_sample_ > FREE_FALL_THRESHOLD)){ // Deploy main backup
      if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup || rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup){
        deploy_2_time_ = 0;
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      }
      flight_stats_->flight_state = FlightStates::kMainBackupDeployed;
      flight_stats_->main_backup_deploy_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->main_backup_deploy_sample_count = flight_stats_->sample_count;
    }
    noseover_time_++;
  }

  if (HAL_GPIO_ReadPin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin) == GPIO_PIN_SET){
    if (deploy_1_time_ >= SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10) // Stop deploy 1 signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
    deploy_1_time_++;
  }

  if (HAL_GPIO_ReadPin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin) == GPIO_PIN_SET){
    if (deploy_2_time_ >= SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10) // Stop deploy 2 signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
    deploy_2_time_++;
  }

  if (flight_stats_->flight_state >= FlightStates::kNoseover && flight_stats_->flight_state < FlightStates::kLanded){ // Landing
    if (abs(velocity_long_sample_) < DESCENT_RATE_THRESHOLD && flight_stats_->agl[flight_stats_->flight_data_array_index] < MAX_LANDING_ALTITUDE){
      flight_stats_->flight_state = FlightStates::kLanded;
      flight_stats_->landing_altitude = flight_stats_->agl[flight_stats_->flight_data_array_index];
      flight_stats_->landing_sample_count = flight_stats_->sample_count;
    }
  }
  mFlightState = flight_stats_->flight_state;
}

void FlightManager::UpdateVelocity(){
  if (flight_stats_->flight_state < FlightStates::kLanded && flight_stats_->flight_data_array_index >= VELOCITY_SAMPLES_LONG - 1){
    float sampleTimeShort = (1.0 - VELOCITY_SAMPLES_SHORT) / (SAMPLES_PER_SECOND * 2);
  	velocity_short_sample_ = 0.0;
    for (int i = flight_stats_->flight_data_array_index - VELOCITY_SAMPLES_SHORT + 1; i <= flight_stats_->flight_data_array_index; i++){
      velocity_short_sample_ += sampleTimeShort * flight_stats_->agl[i];
      sampleTimeShort += 1.0 / SAMPLES_PER_SECOND;
    }
    velocity_short_sample_ /= velocity_short_sum_sq_; //sum of sampleTimes ^ 2
    mVelocityShortSample = velocity_short_sample_;
    velocity_long_sample_ = 0.0;
    float sampleTimeLong = (1.0 - VELOCITY_SAMPLES_LONG) / (SAMPLES_PER_SECOND * 2);
    for (int i = flight_stats_->flight_data_array_index - VELOCITY_SAMPLES_LONG + 1; i <= flight_stats_->flight_data_array_index; i++){
      velocity_long_sample_ += sampleTimeLong * flight_stats_->agl[i];
      sampleTimeLong += 1.0 / SAMPLES_PER_SECOND;
    }
    velocity_long_sample_ /= velocity_long_sum_sq_; //sum of sampleTimes ^ 2
    mVelocityLongSample = velocity_long_sample_;
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

void FlightManager::ResetFlightStats(bool events_only){
  flight_stats_->max_altitude = 0.0;
  flight_stats_->max_altitude_sample_count = 0;
  flight_stats_->launch_detect_altitude = 0.0;
  flight_stats_->launch_detect_sample_count = 0.0;
  flight_stats_->burnout_altitude = 0.0;
  flight_stats_->burnout_sample_count = 0;
  flight_stats_->nose_over_altitude = 0.0;
  flight_stats_->nose_over_sample_count = 0;
  flight_stats_->drogue_primary_deploy_altitude = 0.0;
  flight_stats_->drogue_primary_deploy_sample_count = 0;
  flight_stats_->drogue_backup_deploy_altitude = 0.0;
  flight_stats_->drogue_backup_deploy_sample_count = 0;
  flight_stats_->main_primary_deploy_altitude = 0.0;
  flight_stats_->main_primary_deploy_sample_count = 0;
  flight_stats_->main_backup_deploy_altitude = 0.0;
  flight_stats_->main_backup_deploy_sample_count = 0;
  flight_stats_->landing_altitude = 0.0;
  flight_stats_->landing_sample_count = 0;
  if (!events_only){
    flight_stats_->launch_date = 0;
    flight_stats_->launch_time = 0;
    flight_stats_->sample_count = 0;
    flight_stats_->g_range_scale = 0.0;
    flight_stats_->flight_state = kWaitingLaunch;
    flight_stats_->agl_adjust = 0.0;
    flight_stats_->flight_data_array_index = 0;
    flight_stats_->test_data_sample_count = 0;
    for (int i = 0; i < FLIGHT_DATA_ARRAY_SIZE; i++){
      flight_stats_->agl[i] = 0.0;
      flight_stats_->accelerometer[i].x = 0;
      flight_stats_->accelerometer[i].y = 0;
      flight_stats_->accelerometer[i].z = 0;
    }
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
