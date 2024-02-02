#include "RocketGPS.hpp"
#include "sys_app.h"
#include "stdio.h"

RocketGPS::RocketGPS(){
}

void RocketGPS::Begin(){
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n", 51, UART_TIMEOUT);
	if (status != HAL_OK)
	  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
	status = HAL_UART_Transmit(&huart1, (uint8_t*)"$PMTK183*38\r\n", 13, UART_TIMEOUT);
	if (status != HAL_OK)
	  HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
}

void RocketGPS::ProcessGgaSentence(){
	while (gga_write_);
	uint8_t checksum = 0;
	bool checksumCaptureActive = false;
	memset(&telemetry_data_, 0x0, sizeof(telemetry_data_));
	int i = 0, iPrev = 0, field = 0;
	while (i < gga_sentence_index_){
		if (gga_sentence_[i] == ','){
			switch (field){
				case 0:
					memcpy(telemetry_data_.sentence_type, gga_sentence_, GPS_SENTENCE_TYPE_LEN);
					break;
				case 1:
					gga_sentence_[i] = 0;
					telemetry_data_.time_stamp = atoi((const char*)&gga_sentence_[iPrev + 1]);
					break;
				case 2:
					gga_sentence_[i] = 0;
					telemetry_data_.latitude = atof((const char*)&gga_sentence_[iPrev + 1]);
					break;
				case 3:
					if (gga_sentence_[iPrev + 1] == 'S')
						telemetry_data_.latitude = -telemetry_data_.latitude;
					break;
				case 4:
					gga_sentence_[i] = 0;
					telemetry_data_.longitude = atof((const char*)&gga_sentence_[iPrev + 1]);
					break;
				case 5:
					if (gga_sentence_[iPrev + 1] == 'W')
						telemetry_data_.longitude = -telemetry_data_.longitude;
					break;
				case 6:
					if (gga_sentence_[iPrev + 1] != ',')
						telemetry_data_.q_ind = gga_sentence_[iPrev + 1];
					break;
				case 7:
					gga_sentence_[i] = 0;
					telemetry_data_.satellites = atoi((const char*)&gga_sentence_[iPrev + 1]);
					break;
				case 8:
					gga_sentence_[i] = 0;
					telemetry_data_.hdop = atof((const char*)&gga_sentence_[iPrev + 1]);
					break;
				case 9:
					gga_sentence_[i] = 0;
					telemetry_data_.altitude = atof((const char*)&gga_sentence_[iPrev + 1]); //To do: populate with altimeter reading
					break;/*
				case 10:
					if (gpsSentence[iPrev + 1] != ',')
						gga.altUnits = gpsSentence[iPrev + 1];
					break;
				case 11:
					gpsSentence[i] = 0;
					gga.geoidalSep = atof((const char*)&gpsSentence[iPrev + 1]);
					break;
				case 12:
					if (gpsSentence[iPrev + 1] != ',')
						gga.geoUnits = gpsSentence[iPrev + 1];
					break;*/
			}
			iPrev = i;
			field++;
		}
		if (gga_sentence_[i] == '*'){
			checksumCaptureActive = false;
			checksum ^= gga_sentence_[i - 1];
		}
		if (checksumCaptureActive)
			checksum ^= gga_sentence_[i];
		if (gga_sentence_[i] == '$')
			checksumCaptureActive = true;
		i++;
	}
	memcpy(telemetry_data_.checksum, &gga_sentence_[iPrev + 1 + (telemetry_data_.q_ind == '2' ? 4 : 0)], GPS_SENTENCE_CHECKSUM_LEN);
}

void RocketGPS::ProcessRmcSentence(){
	while (rmc_write_);
	int i = 0, iPrev = 0, field = 0;
	while (i < rmc_sentence_index_){
		if (rmc_sentence_[i] == ','){
			switch (field){
				case 9:
					rmc_sentence_[i] = 0;
					telemetry_data_.date_stamp = atoi((const char*)&rmc_sentence_[iPrev + 1]);
					break;
			}
			iPrev = i;
			field++;
		}
		i++;
	}
}

void RocketGPS::ProcessChar(uint8_t gps_char){
	bool ack = false, other_msg = false, gll_msg = false, gsa_msg = false, gsv_msg = false, vtg_msg = false;
	if ((gps_msg_buffer_[gps_msg_buffer_index_++] = gps_char) == '\n'){
		if (strncmp((char*)gps_msg_buffer_ + 3, "GGA", GPS_SENTENCE_TYPE_LEN - 3) == 0){
			gga_write_ = true;
			memcpy(gga_sentence_, gps_msg_buffer_, RX_BUFFER_SIZE);
			gga_sentence_index_ = gps_msg_buffer_index_;
		}
		else if (strncmp((char*)gps_msg_buffer_ + 3, "RMC", GPS_SENTENCE_TYPE_LEN - 3) == 0){
			rmc_write_ = true;
		  //APP_LOG(TS_ON, VLEVEL_L, "RMC Message\n\r");
			memcpy(rmc_sentence_, gps_msg_buffer_, RX_BUFFER_SIZE);
			rmc_sentence_index_ = gps_msg_buffer_index_;
		}
		else if (strncmp((char*)gps_msg_buffer_ + 3, "GLL", GPS_SENTENCE_TYPE_LEN - 3) == 0){
			gll_msg = true;
		}
		else if (strncmp((char*)gps_msg_buffer_ + 3, "GSA", GPS_SENTENCE_TYPE_LEN - 3) == 0){
			gsa_msg = true;
		}
		else if (strncmp((char*)gps_msg_buffer_ + 3, "GSV", GPS_SENTENCE_TYPE_LEN - 3) == 0){
			gsv_msg = true;
		}
		else if (strncmp((char*)gps_msg_buffer_ + 3, "VTG", GPS_SENTENCE_TYPE_LEN - 3) == 0){
			vtg_msg = true;
		}
		else if (strncmp((char*)gps_msg_buffer_ + 1, "PMTK", 4) == 0){
			ack = true;
		}
		else{
			other_msg = true;
		}
		gps_msg_buffer_index_ = 0;
	}
}

void RocketGPS::GgaToPacket(uint8_t *packet){
	memcpy(packet, &telemetry_data_, sizeof(telemetry_data_));
}

uint8_t RocketGPS::GgaSize(){
	return sizeof(telemetry_data_);
}

void RocketGPS::SetFlightState(uint8_t flight_state){
	telemetry_data_.flight_state = flight_state;
}
