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

#define LORA_MSG_TYPE_SIZE 3
#define FLIGHT_STATS_MSG_SIZE 81

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
  FlightStats flight_stats_;

  const char* lora_startup_message_ = "Rocket Locator v1.2\n\0";
  const char* bad_gps_data_ = "Bad GPS Data\r\n";

  int peripheral_interrupt_count_ = 0;
  int battery_level_ = 0;
  int flight_stats_delay_count_ = 0;

  uint8_t pre_launch_msg_[LORA_MSG_TYPE_SIZE
          + sizeof(GPSData) // GPS data
          + sizeof(uint8_t) // Altimeter initialization status
          + sizeof(uint16_t) // AGL
          + sizeof(uint8_t) // Accelerometer initialization status
          + sizeof(Accelerometer_t) // Raw accelerometer x, y, z values
          + sizeof(uint8_t) // Deployment configuration (drogue primary / drogue backup / main primary / main backup) and status
          + sizeof(uint16_t) // Launch detect altitude
          + sizeof(uint16_t) // Deploy channel 1 trigger threshold
          + sizeof(uint16_t) // Deploy channel 2 trigger threshold
          + sizeof(uint16_t) // Deploy signal duration
          + DEVICE_NAME_LENGTH]
          = {'P', 'R', 'E'};
  uint8_t telemetry_msg_[LORA_MSG_TYPE_SIZE
          + sizeof(GPSData) // GPS data
          + sizeof(flight_stats_.flight_state) // Flight state
          + sizeof(flight_stats_.sample_count) // Sample count
          + SAMPLES_PER_SECOND * sizeof(uint16_t)] // Altimeter data
          = {'T', 'L', 'M'};
  uint8_t flight_stats_msg_[LORA_MSG_TYPE_SIZE + FLIGHT_STATS_MSG_SIZE] = {'F', 'S', 'M'};

  uint8_t config_cycle_count_ = 0;

  bool archive_opened_ = false;
  bool datestamp_saved_ = false;
  bool altimeter_archive_closed_ = false;
  bool accelerometer_archive_closed_ = false;

  EX_Error accelerometer_status_;
  bool altimeter_init_status_;

  void ConfigDevice(float x_axis, float y_axis, float z_axis);
  void SendPreLaunchData();
  void SendTelemetryData();
  void TransmitLEDsOn();/*
  void DisplayDroguePrimaryDrogueBackup();
  void DisplayMainPrimaryMainBackup();
  void DisplayDroguePrimaryMainPrimary();
  void DisplayDrogueBackupMainBackup();
  void SetDisplayDeployMode();
  void ResetDisplayDeployMode();*/
};

extern volatile int m_rocket_service_state;
extern enum DeviceState device_state_;
extern volatile uint8_t m_radio_send;
extern volatile uint8_t m_bad_gps_message;

#endif
