#include "RocketGPS.hpp"

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
  calculated_checksum_ = 0;
  bool checksumCaptureActive = false;
  ResetTelemetryData();
  int i = 0, iPrev = 0, field = 0;
  while (i < gga_sentence_length_){
    if (gga_sentence_[i] == ','){
      switch (field){
        case 0:
          //memcpy(gps_data_.sentence_type, gga_sentence_, GPS_SENTENCE_TYPE_LEN);
          break;
        case 1:
          gga_sentence_[i] = 0;
          gps_data_.time_stamp = atoi((const char*)&gga_sentence_[iPrev + 1]);
          gps_time_valid_ = true;
          break;
        case 2:
          gga_sentence_[i] = 0;
          gps_data_.latitude = atof((const char*)&gga_sentence_[iPrev + 1]);
          break;
        case 3:
          if (gga_sentence_[iPrev + 1] == 'S')
            gps_data_.latitude = -gps_data_.latitude;
          break;
        case 4:
          gga_sentence_[i] = 0;
          gps_data_.longitude = atof((const char*)&gga_sentence_[iPrev + 1]);
          break;
        case 5:
          if (gga_sentence_[iPrev + 1] == 'W')
              gps_data_.longitude = -gps_data_.longitude;
          break;
        case 6:
          if (gga_sentence_[iPrev + 1] != ',')
              gps_data_.q_ind = gga_sentence_[iPrev + 1];
          break;
        case 7:
          gga_sentence_[i] = 0;
          gps_data_.satellites = atoi((const char*)&gga_sentence_[iPrev + 1]);
          break;
        case 8:
          gga_sentence_[i] = 0;
          gps_data_.hdop = atof((const char*)&gga_sentence_[iPrev + 1]);
          break;
        case 9:
          gga_sentence_[i] = 0;
          gps_data_.altitude = atof((const char*)&gga_sentence_[iPrev + 1]);
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
    if (gga_sentence_[i] == '*')
      checksumCaptureActive = false;
    if (checksumCaptureActive)
      calculated_checksum_ ^= gga_sentence_[i];
    if (gga_sentence_[i] == '$')
      checksumCaptureActive = true;
    i++;
  }
  memcpy(gps_data_.checksum, &gga_sentence_[iPrev + 1 + (gps_data_.q_ind == '2' ? 4 : 0)], GPS_SENTENCE_CHECKSUM_LEN);
  gps_checksum_ = (gps_data_.checksum[1] <= '9' ? gps_data_.checksum[1] - '0' : gps_data_.checksum[1] - 55) * 16
      + (gps_data_.checksum[2] <= '9' ? gps_data_.checksum[2] - '0' : gps_data_.checksum[2] - 55);
}

void RocketGPS::ProcessRmcSentence(){
  int i = 0, iPrev = 0, field = 0;
  while (i < rmc_sentence_length_){
    if (rmc_sentence_[i] == ','){
      switch (field){
        case 9:
          rmc_sentence_[i] = 0;
          gps_data_.date_stamp = atoi((const char*)&rmc_sentence_[iPrev + 1]);
          gps_date_valid_ = true;
          break;
      }
      iPrev = i;
      field++;
    }
    i++;
  }
}

void RocketGPS::ResetTelemetryData(){
  gps_data_.time_stamp = 0;
  gps_data_.latitude = 0.0;
  gps_data_.longitude = 0.0;
  gps_data_.q_ind = 0;
  gps_data_.satellites = 0;
  gps_data_.hdop = 0.0;
  gps_data_.altitude = 0.0;
  gps_data_.checksum[0] = 0;
  gps_data_.checksum[1] = 0;
  gps_data_.checksum[2] = 0;
}

void RocketGPS::ProcessChar(uint8_t gps_char){
  if (gps_char == '$')
    gps_msg_buffer_index_ = 0;
  gps_msg_buffer_[gps_msg_buffer_index_++] = gps_char;
  if (gps_msg_buffer_index_ == 7 && strncmp((char*)gps_msg_buffer_ + 3, "GGA", GPS_SENTENCE_TYPE_LEN - 3) == 0)
    m_processing_new_gga_sentence_ = 2;
  if (gps_msg_buffer_index_ == 7 && strncmp((char*)gps_msg_buffer_ + 3, "RMC", GPS_SENTENCE_TYPE_LEN - 3) == 0)
    m_processing_new_rmc_sentence_ = 3;
  if (gps_char == '\n'){
    if (strncmp((char*)gps_msg_buffer_ + 3, "GGA", GPS_SENTENCE_TYPE_LEN - 3) == 0){
      memcpy(gga_sentence_, gps_msg_buffer_, RX_BUFFER_SIZE);
      gga_sentence_length_ = gps_msg_buffer_index_;
      ProcessGgaSentence();
      m_processing_new_gga_sentence_ = 0;
    }
    else if (strncmp((char*)gps_msg_buffer_ + 3, "RMC", GPS_SENTENCE_TYPE_LEN - 3) == 0){
      memcpy(rmc_sentence_, gps_msg_buffer_, RX_BUFFER_SIZE);
      rmc_sentence_length_ = gps_msg_buffer_index_;
      ProcessRmcSentence();
      m_processing_new_rmc_sentence_ = 0;
    }
    else{
      int i = 0;
      i++;
    }
    gps_msg_buffer_index_ = 0;
  }
}

void RocketGPS::GPSToPacket(uint8_t *packet){
  memcpy(packet, &gps_data_, sizeof(gps_data_));
}

int RocketGPS::GetDate(){
  return gps_data_.date_stamp;
}

int RocketGPS::GetTime(){
  return gps_data_.time_stamp;
}

bool RocketGPS::GPSDatestampValid(){
  return gps_date_valid_ && gps_time_valid_;
}

bool RocketGPS::GPSDataValid(){
  return calculated_checksum_ == gps_checksum_;
}
