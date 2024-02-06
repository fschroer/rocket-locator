#include "FlightManager.hpp"
#include "main.h"
#include "math.h"
#include "sys_app.h"

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
  accelerometer_.UpdateAccelerometerValues(&x_axis, &y_axis, &z_axis);
  g_force_short_sample_ = sqrt(x_axis * x_axis + y_axis * y_axis + z_axis * z_axis);
  for (uint8_t i = 0; i < std::extent<decltype(accelerometer_history_)>::value; i++)
    accelerometer_history_[i] = g_force_short_sample_;
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
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
}

void FlightManager::FlightService(){
	GetAltimeterData();
	UpdateFlightState();
	if (flight_stats_->flight_state == flightStates::kWaitingLaunch){
		if (flight_stats_->sample_index < LAUNCH_LOOKBACK_SAMPLES - 1)
			flight_stats_->sample_index++;
		else
			for (int i = 0; i < LAUNCH_LOOKBACK_SAMPLES - 1; i++)
				flight_stats_->agl[i] = flight_stats_->agl[i + 1];
	}
	else{
		if (flight_stats_->flight_state < flightStates::kLanded){
			if (flight_stats_->sample_index < AGL_ARRAY_SIZE - 1)
					flight_stats_->sample_index++;
			else
					flight_stats_->flight_state = flightStates::kLanded;
		}
	}
	//serviceBeeper();
}

void FlightManager::GetAltimeterData(){
  GetAGL();
  flight_stats_->agl[flight_stats_->sample_index] = sensor_agl_; // get altitude above ground level
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

void FlightManager::GetAccelerometerData(float *x_axis, float *y_axis, float *z_axis){
  uint8_t shake_count = 0;
  accelerometer_.UpdateAccelerometerValues(&x_axis_, &y_axis_, &z_axis_);
  uint8_t i = 0;
  g_force_short_sample_ = 0;
  g_force_long_sample_ = 0;
  for (; i < std::extent<decltype(accelerometer_history_)>::value - 1; i++){
    accelerometer_history_[i] = accelerometer_history_[i + 1];
    if (i >= std::extent<decltype(accelerometer_history_)>::value - 3)
    g_force_short_sample_ += accelerometer_history_[i];
    g_force_long_sample_ += accelerometer_history_[i];
  }
  accelerometer_history_[i] = sqrt(x_axis_ * x_axis_ + y_axis_ * y_axis_ + z_axis_ * z_axis_);
  m_g_force = sqrt(x_axis_ * x_axis_ + y_axis_ * y_axis_ + z_axis_ * z_axis_);
  g_force_short_sample_ += accelerometer_history_[i];
  g_force_short_sample_ /= G_FORCE_SAMPLES_SHORT;
  g_force_long_sample_ += accelerometer_history_[i];
  g_force_long_sample_ /= G_FORCE_SAMPLES_LONG;
  for (uint8_t j = 0; j <= i; j++)
    if (accelerometer_history_[j] > g_force_short_sample_ + 0.25)
      shake_count++;
  if (shake_count > 7)
    device_shake_ = true;
  else
    device_shake_ = false;
  *x_axis = x_axis_;
  *y_axis = y_axis_;
  *z_axis = z_axis_;
  mX = x_axis_;
  mY = y_axis_;
  mZ = z_axis_;
}

Accelerometer_t FlightManager::GetRawAccelerometerData(){
  return accelerometer_.GetRawAccelerometerValues();
}

void FlightManager::UpdateFlightState(){ // Update flight state
  if (flight_stats_->flight_state == flightStates::kWaitingLaunch && flight_stats_->agl[flight_stats_->sample_index] > rocket_settings_->launch_detect_altitude // Detect launch
      && velocity_short_sample_ > LAUNCH_VELOCITY && g_force_short_sample_ > LAUNCH_G_FORCE){ // Minimum altitude, velocity, acceleration
    flight_stats_->flight_state = flightStates::kLaunched;
    flight_stats_->launch_detect_altitude = flight_stats_->agl[flight_stats_->sample_index];
    flight_stats_->launch_detect_sample_index = flight_stats_->sample_index;
    //APP_LOG(TS_OFF, VLEVEL_M, "\r\nLaunch: %d %3.2f %3.2f %3.2f\r\n", flight_stats_->aglIndex, flight_stats_->agl[flight_stats_->aglIndex], velocityShortSample, velocityLongSample);
  }

  if (flight_stats_->flight_state >= flightStates::kLaunched && flight_stats_->flight_state < flightStates::kNoseover && g_force_short_sample_ < 0.5){
    updateMaxAltitude();
    if (flight_stats_->burnout_sample_index == 0){ // Detect burnout
      flight_stats_->flight_state = flightStates::kBurnout;
      flight_stats_->burnout_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->burnout_sample_index = flight_stats_->sample_index;
    }
    float sum1 = 0.0, sum2 = 0.0; // Detect nose over
    for (int i = flight_stats_->sample_index - SAMPLES_PER_SECOND + 1; i < flight_stats_->sample_index - SAMPLES_PER_SECOND / 2 + 1; i++){
      sum1 += flight_stats_->agl[i];
      sum2 += flight_stats_->agl[i + SAMPLES_PER_SECOND / 2];
    }
    if (sum2 <= sum1){
      flight_stats_->nose_over_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->nose_over_sample_index = flight_stats_->sample_index;
      flight_stats_->flight_state = flightStates::kNoseover;
      //APP_LOG(TS_OFF, VLEVEL_M, "\r\nNoseover: %d %3.2f %3.2f %3.2f\r\n", flight_stats_->aglIndex, flight_stats_->agl[flight_stats_->aglIndex], velocityShortSample, velocityLongSample);
    }
  }

  if (flight_stats_->flight_state >= flightStates::kNoseover){
    if (flight_stats_->flight_state < flightStates::kDroguePrimaryDeployed
        && flight_stats_->sample_index >= flight_stats_->nose_over_sample_index + SAMPLES_PER_SECOND * rocket_settings_->drogue_primary_deploy_delay / 10){ // Deploy drogue primary
      if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup || rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary)
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      flight_stats_->drogue_primary_deploy_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->drogue_primary_deploy_sample_index = flight_stats_->sample_index;
      flight_stats_->flight_state = flightStates::kDroguePrimaryDeployed;
    }

    if (flight_stats_->flight_state < flightStates::kDrogueBackupDeployed
        && flight_stats_->sample_index >= flight_stats_->nose_over_sample_index + SAMPLES_PER_SECOND * rocket_settings_->drogue_backup_deploy_delay / 10){ // Deploy drogue backup
      if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup)
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      else if (rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup)
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      flight_stats_->drogue_backup_deploy_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->drogue_backup_deploy_sample_index = flight_stats_->sample_index;
      flight_stats_->flight_state = flightStates::kDrogueBackupDeployed;
    }

    if (flight_stats_->flight_state < flightStates::kMainPrimaryDeployed
        && flight_stats_->agl[flight_stats_->sample_index] <= rocket_settings_->main_primary_deploy_altitude){ // Deploy main primary
      if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup)
        HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_SET);
      else if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary)
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      flight_stats_->flight_state = flightStates::kMainPrimaryDeployed;
      flight_stats_->main_primary_deploy_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->main_primary_deploy_sample_index = flight_stats_->sample_index;
    }

    if (flight_stats_->flight_state < flightStates::kMainBackupDeployed
        && flight_stats_->agl[flight_stats_->sample_index] <= rocket_settings_->main_backup_deploy_altitude){ // Deploy main backup
      if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup || rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup)
        HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_SET);
      flight_stats_->flight_state = flightStates::kMainBackupDeployed;
      flight_stats_->main_backup_deploy_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->main_backup_deploy_sample_index = flight_stats_->sample_index;
    }
  }

  if (HAL_GPIO_ReadPin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin) == GPIO_PIN_SET){
    if ((rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup || rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary)
        && flight_stats_->sample_index >= flight_stats_->drogue_primary_deploy_sample_index + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop drogue primary deployment signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kDroguePrimaryDeployed)
        flight_stats_->flight_state = flightStates::kDroguePrimarySignalOff;
    }
    if (rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup
        && flight_stats_->sample_index >= flight_stats_->drogue_backup_deploy_sample_index + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop drogue backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kDrogueBackupDeployed)
        flight_stats_->flight_state = flightStates::kDrogueBackupSignalOff;
    }
    if (rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup
        && flight_stats_->sample_index >= flight_stats_->main_primary_deploy_sample_index + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main primary deployment signal
      HAL_GPIO_WritePin(DEPLOY_1_GPIO_Port, DEPLOY_1_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kMainPrimaryDeployed)
        flight_stats_->flight_state = flightStates::kMainPrimarySignalOff;
    }
  }

  if (HAL_GPIO_ReadPin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin) == GPIO_PIN_SET){
    if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryDrogueBackup
        && flight_stats_->sample_index >= flight_stats_->drogue_backup_deploy_sample_index + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kDrogueBackupDeployed)
        flight_stats_->flight_state = flightStates::kDrogueBackupSignalOff;
    }
    if (rocket_settings_->deploy_mode == DeployMode::kDroguePrimaryMainPrimary
        && flight_stats_->sample_index >= flight_stats_->main_primary_deploy_sample_index + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kMainPrimaryDeployed)
        flight_stats_->flight_state = flightStates::kMainPrimarySignalOff;
    }
    if ((rocket_settings_->deploy_mode == DeployMode::kMainPrimaryMainBackup || rocket_settings_->deploy_mode == DeployMode::kDrogueBackupMainBackup)
        && flight_stats_->sample_index >= flight_stats_->main_backup_deploy_sample_index + SAMPLES_PER_SECOND * rocket_settings_->deploy_signal_duration / 10){ // Stop main backup deployment signal
      HAL_GPIO_WritePin(DEPLOY_2_GPIO_Port, DEPLOY_2_Pin, GPIO_PIN_RESET);
      if (flight_stats_->flight_state == flightStates::kMainBackupDeployed)
        flight_stats_->flight_state = flightStates::kMainBackupSignalOff;
    }
  }

  if (flight_stats_->flight_state >= flightStates::kNoseover && flight_stats_->flight_state < flightStates::kLanded){ // Landing
    if (abs(velocity_long_sample_) < DESCENT_RATE_THRESHOLD && flight_stats_->agl[flight_stats_->sample_index] < MAX_LANDING_ALTITUDE){
      flight_stats_->flight_state = flightStates::kLanded;
      flight_stats_->landing_altitude = flight_stats_->agl[flight_stats_->sample_index];
      flight_stats_->landing_sample_index = flight_stats_->sample_index;
    }
  }
  mFlightState = flight_stats_->flight_state;
}

void FlightManager::GetAGL(){
	Bmp280ReadFloat(&bmp280_, &temperature_, &pressure_, &humidity_);
	sensor_altitude_ =  (44330 * (1.0 - pow(pressure_ / 101325, 0.1903)));
  sensor_agl_ = sensor_altitude_ - flight_stats_->agl_adjust;
  if (flight_stats_->flight_state > flightStates::kWaitingLaunch)
    if (sensor_agl_ > flight_stats_->agl[flight_stats_->sample_index - 1] + MAX_ALTITUDE_SAMPLE_CHANGE || sensor_agl_ < flight_stats_->agl[flight_stats_->sample_index - 1] - MAX_ALTITUDE_SAMPLE_CHANGE)
      sensor_agl_ = flight_stats_->agl[flight_stats_->sample_index - 1]; // Compensate for spurious values
  //Serial.printf("%4.1f ", sensor_agl_);
}

void FlightManager::UpdateVelocity(){
  if (flight_stats_->flight_state < flightStates::kLanded && flight_stats_->sample_index >= VELOCITY_SAMPLES_LONG - 1){
    float sampleTimeShort = (1.0 - VELOCITY_SAMPLES_SHORT) / (SAMPLES_PER_SECOND * 2);
  	velocity_short_sample_ = 0.0;
    for (int i = flight_stats_->sample_index - VELOCITY_SAMPLES_SHORT + 1; i <= flight_stats_->sample_index; i++){
      velocity_short_sample_ += sampleTimeShort * flight_stats_->agl[i];
      sampleTimeShort += 1.0 / SAMPLES_PER_SECOND;
    }
    velocity_short_sample_ /= velocity_short_sum_sq_; //sum of sampleTimes ^ 2
    velocity_long_sample_ = 0.0;
    float sampleTimeLong = (1.0 - VELOCITY_SAMPLES_LONG) / (SAMPLES_PER_SECOND * 2);
    for (int i = flight_stats_->sample_index - VELOCITY_SAMPLES_LONG + 1; i <= flight_stats_->sample_index; i++){
      velocity_long_sample_ += sampleTimeLong * flight_stats_->agl[i];
      sampleTimeLong += 1.0 / SAMPLES_PER_SECOND;
    }
    velocity_long_sample_ /= velocity_long_sum_sq_; //sum of sampleTimes ^ 2
  }
}

void FlightManager::updateMaxAltitude(){
  if (flight_stats_->agl[flight_stats_->sample_index] > flight_stats_->max_altitude){ // Update max altitude for apogee determination
    flight_stats_->max_altitude = flight_stats_->agl[flight_stats_->sample_index];
    flight_stats_->max_altitude_sample_index = flight_stats_->sample_index;
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

void FlightManager::AglToPacket(uint8_t *packet){
  if (flight_stats_->sample_index >= SAMPLES_PER_SECOND)
    if (flight_stats_->flight_state > flightStates::kWaitingLaunch && flight_stats_->flight_state < flightStates::kLanded)
      memcpy(packet, &flight_stats_->agl[flight_stats_->sample_index - SAMPLES_PER_SECOND], sizeof(float) * SAMPLES_PER_SECOND);
    else
      *packet = flight_stats_->agl[flight_stats_->sample_index - 1];
}

bool FlightManager::DeviceShake(){
  return device_shake_;
}

float FlightManager::GetGRangeScale(){
  return accelerometer_.GetGRangeScale();
}

float FlightManager::GetGForceShortSample(){
  return g_force_short_sample_;
}
