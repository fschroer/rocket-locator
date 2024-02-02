#ifndef FLIGHT_MANAGER
#define FLIGHT_MANAGER

#include "RocketDefs.hpp"
#include "stm32_timer.h"
#include "i2c.h"
#include "bmp280.h"
#include "Accelerometer.hpp"

//#define DROGUE_SENSE_PIN 15
//#define MAIN_SENSE_PIN 5
#define LAUNCH_LOOKBACK_SAMPLES 21 // Must be odd and >= VELOCITY_SAMPLES_LONG
#define LAUNCH_VELOCITY 5.0 // Launch start detection velocity
#define MAX_PRELAUNCH_ALTITUDE_CHANGE 0.5
#define VELOCITY_SAMPLES_LONG 21 // Must be odd
#define VELOCITY_SAMPLES_SHORT 7 // Must be odd
#define MACH_LOCKOUT_VELOCITY 250.0 // Mach = 340.29 m/s
#define DEPLOYMENT_LOCKOUT_ALTITUDE_MAX_CHANGE 100.0 // Maximum altitude change to remove deployment lockout, in meters
#define MAX_LANDING_ALTITUDE 30.0 // Maximum altitude to detect landing
#define MAX_ALTITUDE_SAMPLE_CHANGE 4.5 * 340.29 / SAMPLES_PER_SECOND // Maximum altitude change per sample: Mach 4.5 = fastest amateur rocket velocity
#define DESCENT_RATE_THRESHOLD 0.25 // meters per second

class FlightManager{
public:
  FlightManager();
  FlightManager(RocketSettings *rocket_settings, SensorValues *sensor_values, FlightStats *flight_stats);
  void Begin();
  void GetAccelerometerData(float *x_axis, float *y_axis, float *z_axis);
  void FlightService();
  void AglToPacket(uint8_t *packet);
  bool DeviceShake();
  DeployMode GetDeployMode();
  void SetDeployMode(DeployMode deploy_mode);
  void SaveRocketSettings();
private:
  Accelerometer accelerometer;

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

	float x_axis_ = 0.0, y_axis_ = 0.0, z_axis_ = 0.0;
  bool device_shake_ = false;
  float accelerometer_history_[SAMPLES_PER_SECOND * 3] = {0.0};

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
