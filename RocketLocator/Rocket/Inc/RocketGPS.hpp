#ifndef ROCKET_GPS
#define ROCKET_GPS

//#include "RocketDefs.hpp"
#include "FlightManager.hpp"
#include "stdlib.h"
#include "string.h"
#include "usart.h"
#include "radio_driver.h"

#define GPS_SENTENCE_TYPE_LEN 6
#define GPS_SENTENCE_CHECKSUM_LEN 3
#define GPS_TX_PIN PA3
#define GPS_RX_PIN PA2

struct __attribute__ ((packed)) Telemetry {
  char sentence_type[GPS_SENTENCE_TYPE_LEN] = {0};
  int date_stamp;
  int time_stamp;
  double latitude;
  double longitude;
  char q_ind;
  uint8_t satellites;
  float hdop;
  float altitude;
  char checksum[GPS_SENTENCE_CHECKSUM_LEN] = {0};
  uint8_t flight_state = flightStates::kWaitingLaunch;
};

class RocketGPS{
public:
  Telemetry telemetry_data_;
  bool gga_write_ = false;
  bool rmc_write_ = false;

  RocketGPS();
  void Begin();
  void ProcessChar(uint8_t gps_char);
  void ProcessGgaSentence();
  void ProcessRmcSentence();
  void GgaToPacket(uint8_t *packet);
  uint8_t GgaSize();
  void SetFlightState(uint8_t flight_state);
  int GetDate();
  int GetTime();
private:
  uint16_t gps_msg_buffer_index_ = 0;
  uint8_t gps_msg_buffer_[RX_BUFFER_SIZE];
  uint16_t gga_sentence_index_ = 0;
  uint8_t gga_sentence_[RX_BUFFER_SIZE];
  uint16_t rmc_sentence_index_ = 0;
  uint8_t rmc_sentence_[RX_BUFFER_SIZE];
};
#endif
