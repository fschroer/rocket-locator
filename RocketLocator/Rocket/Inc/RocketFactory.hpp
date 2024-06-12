#ifndef ROCKET_FACTORY
#define ROCKET_FACTORY

#include <RocketDefs.hpp>
#include <FlightManager.hpp>
#include <RocketGPS.hpp>
#include <RocketFile.hpp>
#include <RocketConfig.hpp>
//#include <Speaker.hpp>
//#include "stm32wlxx_hal_rtc.h"
#include "radio.h"

#define BATT_LEVEL_PIN 35
#define FLIGHT_STATS_MSG_HDR_SIZE 3
#define FLIGHT_STATS_MSG_SIZE 81
#define NMEA_MSG_DELAY 5

class RocketFactory{
public:
  RocketFactory();
  void Begin();
  void ProcessRocketEvents(uint8_t rocket_service_count);
  void ProcessUART1Char(uint8_t uart_char);
  void ProcessUART2Char(UART_HandleTypeDef *huart2, uint8_t uart_char);
private:
  FlightManager flight_manager_;
  RocketGPS rocket_gps_;
  RocketFile rocket_file_;
  RocketConfig rocket_config_;
  //Speaker speaker;

  RocketSettings rocket_settings_;
  SensorValues sensor_values_;
  FlightStats flight_stats_;

  const char* lora_startup_message_ = "Rocket Locator v1.2\n\0";

  int peripheral_interrupt_count_ = 0;
  int battery_level_ = 0;
  int flight_stats_delay_count_ = 0;

  uint8_t flight_stats_msg_[FLIGHT_STATS_MSG_HDR_SIZE + FLIGHT_STATS_MSG_SIZE] = {'F', 'S', 'M'};

  uint8_t config_cycle_count_ = 0;

  bool archive_opened_ = false;
  bool datestamp_saved_ = false;
  bool altimeter_archive_closed_ = false;
  bool accelerometer_archive_closed_ = false;

  void ConfigDevice(float x_axis, float y_axis, float z_axis);
  void SendTelemetryData();
  void DisplayDroguePrimaryDrogueBackup();
  void DisplayMainPrimaryMainBackup();
  void DisplayDroguePrimaryMainPrimary();
  void DisplayDrogueBackupMainBackup();
  void SetDisplayDeployMode();
  void ResetDisplayDeployMode();
};

extern volatile int m_rocket_service_state;
extern enum DeviceState device_state_;
extern volatile uint8_t m_radio_send;
extern volatile uint8_t m_bad_gps_message;

#endif
