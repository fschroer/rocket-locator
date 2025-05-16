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
//#include "usart.h"
//#include "subghz_phy_app.h"
#include "tim.h"

#define LORA_MSG_TYPE_SIZE 3
#define FLIGHT_DATA_SEQUENCE_SIZE 1
#define FLIGHT_STATS_MSG_SIZE 81
#define FLIGHT_DATA_MESSAGE_SAMPLES 30
#define FLIGHT_DATA_MESSAGE_SIZE (FLIGHT_DATA_MESSAGE_SAMPLES * (sizeof(uint16_t) + sizeof(Accelerometer_t))) // samples * agl size * accelerometer size

#define ADC_CALIBRATION_TIMEOUT_MS       (   1U)
#define ADC_ENABLE_TIMEOUT_MS            (   1U)
#define ADC_DISABLE_TIMEOUT_MS           (   1U)
#define ADC_STOP_CONVERSION_TIMEOUT_MS   (   1U)
#define ADC_CONVERSION_TIMEOUT_MS        (4000U)

/* Delay between ADC end of calibration and ADC enable.                     */
/* Delay estimation in CPU cycles: Case of ADC enable done                  */
/* immediately after ADC calibration, ADC clock setting slow                */
/* (LL_ADC_CLOCK_ASYNC_DIV32). Use a higher delay if ratio                  */
/* (CPU clock / ADC clock) is above 32.                                     */
#define ADC_DELAY_CALIB_ENABLE_CPU_CYCLES  (LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 32)


/* Definitions of environment analog values */
/* Value of analog reference voltage (Vref+), connected to analog voltage   */
/* supply Vdda (unit: mV).                                                  */
#define VDDA_APPLI                       (3300UL)

/* Definitions of data related to this example */
/* ADC unitary conversion timeout */
/* Considering ADC settings, duration of 1 ADC conversion should always    */
/* be lower than 1ms.                                                      */
#define ADC_UNITARY_CONVERSION_TIMEOUT_MS (   1UL)

/* Init variable out of expected ADC conversion data range */
#define VAR_CONVERTED_DATA_INIT_VALUE    (__LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) + 1)
#define BATTERY_LEVEL_VOLTAGE_DIVIDER_ADJUST 490000 / 390000

enum FlightProfileState
{
  kIdle = 0,
  kMetadataRequested = 1,
  kDataRequested = 2,
};

class RocketFactory{
public:
  RocketFactory();
  void Begin();
  void ProcessRocketEvents(uint8_t rocket_service_count);
  void ProcessIncomingLoRaMessage(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
  void ProcessUART1Char(uint8_t uart_char);
  void ProcessUART2Char(UART_HandleTypeDef *huart2, uint8_t uart_char);
  void ReadyToSend();
private:
  FlightManager flight_manager_;
  RocketGPS rocket_gps_;
  RocketFile rocket_file_;
  RocketConfig rocket_config_;
  //Speaker speaker;

  RocketSettings rocket_settings_;
  FlightStats flight_stats_;

  const char* lora_startup_message_ = "Rocket Locator v1.3.1\r\n\0";
  const char* usb_connected_ = "Disconnect USB cable before arming locator\r\n\0";
  const char* bad_gps_data_ = "Bad GPS Data\r\n\0";

  int peripheral_interrupt_count_ = 0;
  int battery_level_ = 0;
  int flight_stats_delay_count_ = 0;

  bool archive_opened_ = false;
  bool datestamp_saved_ = false;
  bool altimeter_archive_closed_ = false;
  bool accelerometer_archive_closed_ = false;
  bool ready_to_send_ = true;
  FlightProfileState flight_profile_state_ = kIdle;
  uint8_t flight_profile_archive_position_ = 0;

  EX_Error accelerometer_status_;
  bool altimeter_init_status_;

  /* Variables for ADC conversion data */
  __IO uint16_t uhADCxConvertedData = VAR_CONVERTED_DATA_INIT_VALUE; /* ADC group regular conversion data */

  /* Variables for ADC conversion data computation to physical values */
  __IO uint16_t uhADCxConvertedData_Voltage_mVolt = 0UL;  /* Value of voltage calculated from ADC conversion data (unit: mV) */
  uint16_t battery_voltage_mvolt_ = 0UL;

  uint8_t pre_launch_msg_[LORA_MSG_TYPE_SIZE
    + sizeof(GPSData) // GPS data
    + sizeof(uint8_t) // Device status: locator state, altimeter, accelerometer, deployment channel 1, deployment channel 2
    + sizeof(uint16_t) // AGL
    + sizeof(Accelerometer_t) // Raw accelerometer x, y, z values
    + sizeof(rocket_settings_.deployment_channel_1_mode) // Locator configuration
    + sizeof(rocket_settings_.deployment_channel_2_mode)
    + sizeof(rocket_settings_.launch_detect_altitude)
    + sizeof(rocket_settings_.drogue_primary_deploy_delay)
    + sizeof(rocket_settings_.drogue_backup_deploy_delay)
    + sizeof(rocket_settings_.main_primary_deploy_altitude)
    + sizeof(rocket_settings_.main_backup_deploy_altitude)
    + sizeof(rocket_settings_.deploy_signal_duration)
    + DEVICE_NAME_LENGTH + 1
    + sizeof(battery_voltage_mvolt_) // Battery level
    ] = {'P', 'R', 'E'};
  uint8_t telemetry_msg_[LORA_MSG_TYPE_SIZE
    + sizeof(GPSData) // GPS data
    + sizeof(uint8_t) // Device status: locator state, altimeter, accelerometer, deployment channel 1, deployment channel 2
    + sizeof(uint16_t) // AGL
    + sizeof(Accelerometer_t) // Raw accelerometer x, y, z values
    + sizeof(float) // Velocity
    + sizeof(flight_stats_.flight_state) // Flight state
    + sizeof(flight_stats_.sample_count) // Sample count
    //+ SAMPLES_PER_SECOND * sizeof(uint16_t) // Altimeter data
    ] = {'T', 'L', 'M'};
  uint8_t flight_stats_msg_[LORA_MSG_TYPE_SIZE + FLIGHT_STATS_MSG_SIZE] = {'F', 'S', 'M'};
  uint8_t flight_profile_metadata_msg_[LORA_MSG_TYPE_SIZE
    + (sizeof(int) // Date
    + sizeof(int) // Time
    + sizeof(float) // Apogee
    + sizeof(float)) // Time to drogue
    * ARCHIVE_POSITIONS
    ] = {'F', 'P', 'M'};
  uint8_t flight_profile_data_msg_[LORA_MSG_TYPE_SIZE
    + sizeof(uint8_t) // Page sequence ID
    + FLIGHT_DATA_MESSAGE_SIZE
    ] = {'F', 'P', 'D'};

  void ConfigDevice(float x_axis, float y_axis, float z_axis);
  void SendPreLaunchData();
  void SendTelemetryData();
  void SendFlightProfileMetadata();
  void SendFlightProfileData(uint8_t archive_position);
  void TransmitLEDsOn();
  uint16_t GetBatteryLevel();
/*  void DisplayDroguePrimaryDrogueBackup();
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
