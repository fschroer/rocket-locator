#ifndef FLIGHT_MANAGER
#define FLIGHT_MANAGER

#include "RocketDefs.hpp"
#include "bmp280.h"
#include "Accelerometer.hpp"
#include "RocketFile.hpp"
#include "sys_app.h"
#include "math.h"

//#define TEST
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
  void UpdateFlightState(RocketFile rocket_file);
  void IncrementFlightDataQueue();
  void AglToPacket(uint8_t *packet, uint8_t length);
  DeployMode GetDeployMode();
  void SetDeployMode(DeployMode deploy_mode);
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

  float test_agl_[352] = {0.4,0.4,0.5,0.6,0.8,1.2,1.7,2.4,3.2,4.4,5.6,7.3,9.1,11.3,13.6,16.2,19.3,22.7,26.2,29.8,33.5,37.2,41,48.9,52.9,56.9,61,65,69,73.3,77.3,81.5,85.6,89.9,94,98.1,102.4,106.5,110.6,114.8,118.9,123,127,131.1,135.1,139.1,143.1,151,155,159,162.8,166.6,170.5,174.3,178.1,181.8,185.6,189.3,193,196.6,200.1,203.8,207.3,210.8,214.3,217.8,221.1,224.6,227.8,231.3,234.6,237.8,244.3,247.5,250.6,253.6,256.7,259.8,262.8,265.8,268.7,271.7,274.6,277.3,280.2,283.1,285.7,288.5,291.2,293.7,296.5,299,301.6,304.1,306.6,309.1,311.5,316.2,318.6,320.8,323.2,325.5,327.7,329.7,332,334.2,336.2,338.2,340.3,342.5,344.3,346.3,348.3,350.2,352.2,354,355.8,357.7,359.5,361.2,363.1,366.3,368,369.7,371.2,372.8,374.3,375.8,377.3,378.8,380.2,381.7,383.1,384.3,385.7,387.1,388.3,389.6,390.7,392,393.1,394.2,395.3,396.6,397.8,399.2,402.1,403,404,405,405.8,406.7,407.7,408.7,409.7,410.6,411.3,412.1,412.7,413.5,414.1,414.6,415.1,415.6,416,416.5,416.8,417.2,417.6,417.8,418.3,418.6,418.7,419.1,419.2,419.5,419.7,419.8,420,420.2,420.2,420.2,420.3,420.5,420.3,420.5,420.3,420.3,420.3,420.3,420.2,420.2,420.2,420.1,420,419.7,416.5,412.3,410.6,409.3,409,408.6,407.8,406.1,405,403.7,402.7,401.6,400.2,398.8,397.3,397,396.8,396.1,395.2,394.1,394.2,392.7,390.6,389.7,389.7,388.7,388.2,387.5,385.6,384,383.1,382.1,380.7,379,376.6,375.1,375.3,375,374,373.2,372.3,371.5,370.6,368.8,366.7,364.7,363.2,361.2,358.3,357,354.2,351.8,350.6,348.7,346.3,343.8,341.3,339.1,336.7,334.5,332.5,330.2,327.7,324.6,321.5,318.2,315.7,313.1,310.1,307.3,304.2,301.7,298.6,295.3,291.7,288.2,284.7,280.6,277,273.6,270.7,267.6,264.6,261.2,257.5,253.3,249.6,245.5,241.8,238,234.1,230.3,226.5,223,219.8,216.1,212.8,209,205.6,202,198,193.5,189.1,184.1,180.1,176.6,173,170,167.1,163,159.5,156,152,147.8,144.3,140.1,136.6,132.6,129.8,126,122,118,114.1,110.3,106.5,103,99.1,95.5,93,89,86,82.6,79.3,75.5,71.8,67.8,64,60.5,57.2,53.5,50.5,47.2,44.4,41,37.2,33.5,29.6,26.6,23.2,20,16.3,12.8,11.8,11.6,11.6};

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
