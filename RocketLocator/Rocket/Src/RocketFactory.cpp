#include <RocketFactory.hpp>
//#include "RocketDefs.hpp"
//#include "radio.h"

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
  uint8_t hello[] = {'R', 'o', 'c', 'k', 'e', 't', 'L', 'o', 'c', 'a', 't', 'o', 'r', ' ', 'v', '1', '.', '2', '\n'};
  Radio.Send(hello, sizeof(hello));
  flight_manager_.Begin();
  rocket_gps_.Begin();
  SetDisplayDeployMode();
  HAL_Delay(1000);
  ResetDisplayDeployMode();
}

void RocketFactory::ProcessRocketEvents(){
	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
	float x_axis = 0.0, y_axis = 0.0, z_axis = 0.0;
	flight_manager_.GetAccelerometerData(&x_axis, &y_axis, &z_axis);
	switch (device_state_){
		case DeviceState::kRunning:
			flight_manager_.FlightService();
			if (rocket_gps_.gga_write_ && rocket_gps_.rmc_write_){
				rocket_gps_.gga_write_ = false;
				rocket_gps_.rmc_write_ = false;
				if (flight_stats_.flight_state == flightStates::kWaitingLDA) // Blink LoRa transmit LED for visual validation until liftoff
					HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
				if (flight_stats_.flight_state < flightStates::kLanded){ // Send telemetry data only before rocket lands
					SendTelemetryData();
				}
				else{ // Alternate sending telemetry data and flight statistics after rocket lands. Lower frequency conserves battery.
					if (gps_count_ == 0)
						SendTelemetryData();
					else if (gps_count_ == 5){
						memcpy(flight_stats_msg_ + FLIGHT_STATS_MSG_HDR_SIZE, &flight_stats_, FLIGHT_STATS_MSG_SIZE);
						Radio.Send(flight_stats_msg_, FLIGHT_STATS_MSG_HDR_SIZE + FLIGHT_STATS_MSG_SIZE);
					}
					if (++gps_count_ >= 10)
						gps_count_ = 0;
				}
			}
			SetDeviceState();
			break;
		case DeviceState::kConfig:
			//ConfigDevice(x_axis, y_axis, z_axis);
			break;
		case DeviceState::kConfigSavePending:
			rocket_file_.SaveRocketSettings(&rocket_settings_);
		  Radio.SetChannel(902300000 + rocket_settings_.lora_channel * 200000);
			device_state_ = DeviceState::kRunning;
			break;
	}
}

void RocketFactory::SetDeviceState(){
	if (flight_stats_.flight_state == flightStates::kWaitingLDA){
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
		//if (flightStats.flightState > flightStates::WAITING_LDA && flightStats.flightState < flightStates::DROGUE_DEPLOYED){
			uint8_t telemetryPacket[ggaSize + sizeof(float) * SAMPLES_PER_SECOND] = {0};
			rocket_gps_.GgaToPacket(telemetryPacket);
			flight_manager_.AglToPacket(telemetryPacket + ggaSize);
			Radio.Send(telemetryPacket, sizeof(telemetryPacket));/*
		}
		else{
			uint8_t telemetryPacket[ggaSize] = {0};
			rocketGPS.ggaToPacket(telemetryPacket);
			Radio.Send(telemetryPacket, sizeof(telemetryPacket));
		}*/
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin); /* LED_BLUE */
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
