#ifndef FLIGHT_MANAGER
#define FLIGHT_MANAGER

#include "RocketDefs.hpp"
#include "bmp280.h"
#include "Accelerometer.hpp"
#include "sys_app.h"
#include "math.h"

//#define TEST 0
#define AGL_RESET_TIME 60 * SAMPLES_PER_SECOND // Frequency with which altimeter adjustment offsets to ground level
#define LAUNCH_LOOKBACK_SAMPLES 21 // Must be odd and >= VELOCITY_SAMPLES_LONG
#define VELOCITY_SAMPLES_LONG 21 // Must be odd and less than FLIGHT_DATA_ARRAY_SIZE
#define VELOCITY_SAMPLES_SHORT 7 // Must be odd and less than VELOCITY_SAMPLES_LONG
#define LAUNCH_VELOCITY 5.0 // Launch detection velocity threshold
#define MAX_AGL_ADJUST_G_FORCE 1.1 // Maximum G force allowed for AGL adjustment
#define LAUNCH_G_FORCE 1.5 // Launch detection acceleration threshold
#define G_FORCE_SAMPLES_SHORT 3
#define G_FORCE_SAMPLES_LONG 2 * SAMPLES_PER_SECOND // Must be less than FLIGHT_DATA_ARRAY_SIZE
#define FREE_FALL_THRESHOLD 20 // Emergency main deployment occurs if descent rate exceeds this threshold
#define MAX_LANDING_ALTITUDE 30.0 // Maximum altitude to detect landing
//#define MAX_ALTITUDE_SAMPLE_CHANGE 4.5 * 340.29 / SAMPLES_PER_SECOND // Maximum altitude change per sample: Mach 4.5 = fastest amateur rocket velocity
#define DESCENT_RATE_THRESHOLD 0.25 // Landing detection velocity threshold in meters per second

class FlightManager{
public:
  FlightManager();
  FlightManager(RocketSettings *rocket_settings, FlightStats *flight_stats);
  void Begin(EX_Error *accelerometer_init_status, bool *altimeter_init_status);
  void GetAccelerometerData();
  void GetAGL();
  void UpdateVelocity();
  void UpdateFlightState();
  void IncrementFlightDataQueue();
  void AglToPacket(uint8_t *packet, uint8_t length);
  DeployMode GetDeployMode();
  void SetDeployMode(DeployMode deploy_mode);
  void SaveRocketSettings();
  float GetGForceShortSample();
  void ResetFlightStats(bool events_only);

private:
  Accelerometer accelerometer_;

  RocketSettings *rocket_settings_;
  FlightStats *flight_stats_;

  int noseover_time_ = 0;
  int deploy_1_time_ = 0;
  int deploy_2_time_ = 0;

  float velocity_short_sum_sq_ = 0.0;
  float velocity_long_sum_sq_ = 0.0;
  float velocity_short_sample_ = 0.0;
  float velocity_long_sample_ = 0.0;

  BMP280_HandleTypedef bmp280_;
  float pressure_, temperature_, humidity_;
  float sensor_altitude_ = 0.0;
  int agl_adjust_count_ = 0;

  float g_force_last_ = 0.0, g_force_short_sample_ = 0.0, g_force_long_sample_;
  uint8_t accelerometer_state_ = AccelerometerStates::kAtRest;

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
extern volatile int m_rocket_service_state;

#endif
