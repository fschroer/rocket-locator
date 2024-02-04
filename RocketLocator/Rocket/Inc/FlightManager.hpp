#ifndef FLIGHT_MANAGER
#define FLIGHT_MANAGER

#include "RocketDefs.hpp"
#include "RocketFile.hpp"
#include "stm32_timer.h"
#include "i2c.h"
#include "bmp280.h"
#include "Accelerometer.hpp"

//#define DROGUE_SENSE_PIN 15
//#define MAIN_SENSE_PIN 5
#define AGL_RESET_TIME_SECONDS 60 // Frequency with which altimeter adjustment offsets to ground level
#define LAUNCH_LOOKBACK_SAMPLES 21 // Must be odd and >= VELOCITY_SAMPLES_LONG
#define VELOCITY_SAMPLES_LONG 21 // Must be odd
#define VELOCITY_SAMPLES_SHORT 7 // Must be odd
#define LAUNCH_VELOCITY 5.0 // Launch start detection velocity
#define LAUNCH_G_FORCE 2.0 // Launch acceleration
#define G_FORCE_SAMPLES_SHORT 3
#define G_FORCE_SAMPLES_LONG 3 * SAMPLES_PER_SECOND
#define MACH_LOCKOUT_VELOCITY 250.0 // Mach = 340.29 m/s
#define DEPLOYMENT_LOCKOUT_ALTITUDE_MAX_CHANGE 100.0 // Maximum altitude change to remove deployment lockout, in meters
#define MAX_LANDING_ALTITUDE 30.0 // Maximum altitude to detect landing
#define MAX_ALTITUDE_SAMPLE_CHANGE 4.5 * 340.29 / SAMPLES_PER_SECOND // Maximum altitude change per sample: Mach 4.5 = fastest amateur rocket velocity
#define DESCENT_RATE_THRESHOLD 0.25 // meters per second

enum flightStates
{
  kWaitingLaunch = 0,
  kLaunched = 1,
  kMachLockoutEntered = 2,
  kMachLockoutReleased = 3,
  kBurnout = 4,
  kNoseover = 5,
  kDroguePrimaryDeployed = 6,
  kDroguePrimarySignalOff = 7,
  kDrogueBackupDeployed = 8,
  kDrogueBackupSignalOff = 9,
  kMainPrimaryDeployed = 10,
  kMainPrimarySignalOff = 11,
  kMainBackupDeployed = 12,
  kMainBackupSignalOff = 13,
  kLanded = 14
};

struct FlightStats {
  //bool armed; //deployment system enabled state
  //bool prevArmed;
  uint8_t flight_state = flightStates::kWaitingLaunch;
  //int prevFlightState = flightStates::WAITING_LDA;
  float agl_adjust = 0.0;
  float max_altitude;
  int max_altitude_time;
  float launch_detect_altitude;
  int launch_detect_time;
  float mach_lockout_entered_altitude;
  int mach_lockout_entered_time;
  float burnout_altitude;
  int burnout_time;
  float mach_lockout_released_altitude;
  int mach_lockout_released_time;
  float nose_over_altitude;
  int nose_over_time;
  float drogue_primary_deploy_altitude;
  int drogue_primary_deploy_time;
  float drogue_backup_deploy_altitude;
  int drogue_backup_deploy_time;
  float main_primary_deploy_altitude;
  int main_primary_deploy_time;
  float main_backup_deploy_altitude;
  int main_backup_deploy_time;
  float land_altitude;
  int land_time;
  int sample_index;
  float agl[AGL_ARRAY_SIZE] = {0.0};
};

class FlightManager{
public:
  FlightManager();
  FlightManager(RocketSettings *rocket_settings, SensorValues *sensor_values, FlightStats *flight_stats);
  void Begin();
  void GetAccelerometerData(float *x_axis, float *y_axis, float *z_axis);
  Accelerometer_t GetRawAccelerometerData();
  void FlightService();
  void AglToPacket(uint8_t *packet);
  bool DeviceShake();
  DeployMode GetDeployMode();
  void SetDeployMode(DeployMode deploy_mode);
  void SaveRocketSettings();
  float GetGRangeScale();
  float GetGForceShortSample();
private:
  Accelerometer accelerometer_;

  RocketSettings *rocket_settings_;
  SensorValues *sensor_values_;
  FlightStats *flight_stats_;

  int mach_release_count_ = 0;
  float velocity_short_sum_sq_ = 0.0;
  float velocity_long_sum_sq_ = 0.0;
  float velocity_short_sample_ = 0.0;
  float velocity_long_sample_ = 0.0;

  BMP280_HandleTypedef bmp280_;
  float pressure_, temperature_, humidity_;
  float sensor_altitude_ = 0.0;
  float sensor_agl_ = 0.0;
  int agl_adjust_count_ = 0;

  float x_axis_ = 0.0, y_axis_ = 0.0, z_axis_ = 0.0, g_force_short_sample_ = 0.0, g_force_long_sample_;
  bool device_shake_ = false;
  float accelerometer_history_[G_FORCE_SAMPLES_LONG] = {0.0};

  void GetAltimeterData();
  void UpdateFlightState();
  void GetAGL();
  void UpdateVelocity();
  void updateMaxAltitude();
  void SumSquares();
  void serviceBeeper();
};

extern volatile float mAGL;
extern volatile float mVelocityShortSample;
extern volatile float mVelocityLongSample;
extern volatile int mFlightState;
extern volatile DeployMode mDeployMode;
extern volatile float mX, mY, mZ;
extern volatile float m_g_force;

#endif
