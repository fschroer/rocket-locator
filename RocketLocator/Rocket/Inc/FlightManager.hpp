#ifndef FLIGHT_MANAGER
#define FLIGHT_MANAGER

#include "RocketDefs.hpp"
#include "RocketFile.hpp"
#include "bmp280.h"
#include "Accelerometer.hpp"

#define TEST 1
//#define DROGUE_SENSE_PIN 15
//#define MAIN_SENSE_PIN 5
#define AGL_RESET_TIME_SECONDS 60 // Frequency with which altimeter adjustment offsets to ground level
#define LAUNCH_LOOKBACK_SAMPLES 21 // Must be odd and >= VELOCITY_SAMPLES_LONG
#define VELOCITY_SAMPLES_LONG 21 // Must be odd and less than FLIGHT_DATA_ARRAY_SIZE
#define VELOCITY_SAMPLES_SHORT 7 // Must be odd and less than VELOCITY_SAMPLES_LONG
#define LAUNCH_VELOCITY 5.0 // Launch start detection velocity
#define LAUNCH_G_FORCE 2.0 // Launch acceleration
#define G_FORCE_SAMPLES_SHORT 3
#define G_FORCE_SAMPLES_LONG 2 * SAMPLES_PER_SECOND // Must be less than FLIGHT_DATA_ARRAY_SIZE
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
  void GetAccelerometerData();
  void FlightService();
  void AglToPacket(uint8_t *packet);
  DeployMode GetDeployMode();
  void SetDeployMode(DeployMode deploy_mode);
  void SaveRocketSettings();
  float GetGForceShortSample();

  uint16_t test_data_sample_count_ = 0;
  float test_agl_[1200] = {0.66, 1.18, 1.84, 2.65, 3.61, 4.72, 5.98, 7.39, 8.95, 10.65, 12.51, 14.52, 16.68, 18.98, 21.44, 24.05, 26.81, 29.71, 32.77, 35.98, 39.34, 42.85, 46.51, 50.31, 54.27, 58.38, 62.63, 67.03, 71.59, 76.28, 81.13, 86.13, 91.27, 96.56, 101.99, 107.57, 113.3, 119.17, 125.18, 131.34, 137.64, 144.08, 150.67, 157.39, 164.26, 171.27, 178.41, 185.7, 192.98, 200.17, 207.29, 214.37, 221.39, 228.36, 235.28, 242.15, 248.97, 255.73, 262.45, 269.12, 275.73, 282.3, 288.82, 295.29, 301.72, 308.09, 314.42, 320.7, 326.94, 333.13, 339.27, 345.37, 351.42, 357.42, 363.39, 369.3, 375.18, 381.01, 386.79, 392.54, 398.24, 403.9, 409.51, 415.08, 420.61, 426.1, 431.55, 436.96, 442.33, 447.65, 452.94, 458.18, 463.39, 468.55, 473.68, 478.76, 483.81, 488.82, 493.79, 498.72, 503.62, 508.47, 513.29, 518.07, 522.82, 527.52, 532.19, 536.83, 541.42, 545.98, 550.51, 554.99, 559.45, 563.86, 568.24, 572.59, 576.9, 581.18, 585.42, 589.63, 593.8, 597.94, 602.04, 606.11, 610.15, 614.15, 618.12, 622.05, 625.96, 629.83, 633.66, 637.47, 641.24, 644.98, 648.68, 652.36, 656, 659.61, 663.19, 666.74, 670.25, 673.73, 677.19, 680.61, 684, 687.36, 690.69, 693.99, 697.25, 700.49, 703.7, 706.87, 710.02, 713.13, 716.22, 719.28, 722.3, 725.3, 728.27, 731.21, 734.11, 736.99, 739.84, 742.66, 745.46, 748.22, 750.95, 753.66, 756.34, 758.99, 761.61, 764.2, 766.76, 769.3, 771.81, 774.28, 776.74, 779.16, 781.56, 783.92, 786.26, 788.58, 790.86, 793.12, 795.35, 797.55, 799.73, 801.88, 804, 806.09, 808.16, 810.2, 812.22, 814.2, 816.16, 818.1, 820.01, 821.89, 823.74, 825.57, 827.37, 829.15, 830.89, 832.62, 834.31, 835.98, 837.63, 839.25, 840.84, 842.4, 843.94, 845.46, 846.95, 848.41, 849.85, 851.26, 852.65, 854.01, 855.34, 856.65, 857.93, 859.19, 860.43, 861.63, 862.82, 863.97, 865.1, 866.21, 867.29, 868.35, 869.38, 870.39, 871.37, 872.32, 873.25, 874.16, 875.04, 875.9, 876.73, 877.53, 878.31, 879.07, 879.8, 880.51, 881.19, 881.85, 882.48, 883.09, 883.67, 884.23, 884.76, 885.27, 885.76, 886.21, 886.65, 887.06, 887.45, 887.81, 888.14, 888.45, 888.74, 889, 889.24, 889.46, 889.65, 889.81, 889.95, 890.07, 890.16, 890.22, 890.26, 890.28, 890.27, 890.24, 890.19, 890.11, 890, 889.87, 889.72, 889.54, 889.33, 889.11, 887.97, 886.83, 885.69, 884.56, 883.42, 882.28, 881.15, 880.01, 878.87, 877.73, 876.6, 875.46, 874.32, 873.19, 872.05, 870.91, 869.78, 868.64, 867.5, 866.36, 865.23, 864.09, 862.95, 861.82, 860.68, 859.54, 858.41, 857.27, 856.13, 854.99, 853.86, 852.72, 851.58, 850.45, 849.31, 848.17, 847.04, 845.9, 844.76, 843.62, 842.49, 841.35, 840.21, 839.08, 837.94, 836.8, 835.66, 834.53, 833.39, 832.25, 831.12, 829.98, 828.84, 827.71, 826.57, 825.43, 824.29, 823.16, 822.02, 820.88, 819.75, 818.61, 817.47, 816.34, 815.2, 814.06, 812.92, 811.79, 810.65, 809.51, 808.38, 807.24, 806.1, 804.97, 803.83, 802.69, 801.55, 800.42, 799.28, 798.14, 797.01, 795.87, 794.73, 793.6, 792.46, 791.32, 790.18, 789.05, 787.91, 786.77, 785.64, 784.5, 783.36, 782.22, 781.09, 779.95, 778.81, 777.68, 776.54, 775.4, 774.27, 773.13, 771.99, 770.85, 769.72, 768.58, 767.44, 766.31, 765.17, 764.03, 762.9, 761.76, 760.62, 759.48, 758.35, 757.21, 756.07, 754.94, 753.8, 752.66, 751.53, 750.39, 749.25, 748.11, 746.98, 745.84, 744.7, 743.57, 742.43, 741.29, 740.15, 739.02, 737.88, 736.74, 735.61, 734.47, 733.33, 732.2, 731.06, 729.92, 728.78, 727.65, 726.51, 725.37, 724.24, 723.1, 721.96, 720.83, 719.69, 718.55, 717.41, 716.28, 715.14, 714, 712.87, 711.73, 710.59, 709.46, 708.32, 707.18, 706.04, 704.91, 703.77, 702.63, 701.5, 700.36, 699.22, 698.08, 696.95, 695.81, 694.67, 693.54, 692.4, 691.26, 690.13, 688.99, 687.85, 686.71, 685.58, 684.44, 683.3, 682.17, 681.03, 679.89, 678.76, 677.62, 676.48, 675.34, 674.21, 673.07, 671.93, 670.8, 669.66, 668.52, 667.38, 666.25, 665.11, 663.97, 662.84, 661.7, 660.56, 659.43, 658.29, 657.15, 656.01, 654.88, 653.74, 652.6, 651.47, 650.33, 649.19, 648.05, 646.92, 645.78, 644.64, 643.51, 642.37, 641.23, 640.1, 638.96, 637.82, 636.68, 635.55, 634.41, 633.27, 632.14, 631, 629.86, 628.72, 627.59, 626.45, 625.31, 624.18, 623.04, 621.9, 620.77, 619.63, 618.49, 617.35, 616.22, 615.08, 613.94, 612.81, 611.67, 610.53, 609.39, 608.26, 607.12, 605.98, 604.85, 603.71, 602.57, 601.44, 600.3, 599.16, 598.02, 596.89, 595.75, 594.61, 593.48, 592.34, 591.2, 590.06, 588.93, 587.79, 586.65, 585.52, 584.38, 583.24, 582.11, 580.97, 579.83, 578.69, 577.56, 576.42, 575.28, 574.15, 573.01, 571.87, 570.73, 569.6, 568.46, 567.32, 566.19, 565.05, 563.91, 562.78, 561.64, 560.5, 559.36, 558.23, 557.09, 555.95, 554.82, 553.68, 552.54, 551.4, 550.27, 549.13, 547.99, 546.86, 545.72, 544.58, 543.45, 542.31, 541.17, 540.03, 538.9, 537.76, 536.62, 535.49, 534.35, 533.21, 532.08, 530.94, 529.8, 528.66, 527.53, 526.39, 525.25, 524.12, 522.98, 521.84, 520.7, 519.57, 518.43, 517.29, 516.16, 515.02, 513.88, 512.75, 511.61, 510.47, 509.33, 508.2, 507.06, 505.92, 504.79, 503.65, 502.51, 501.37, 500.24, 499.1, 497.96, 496.83, 495.69, 494.55, 493.42, 492.28, 491.14, 490, 488.87, 487.73, 486.59, 485.46, 484.32, 483.18, 482.04, 480.91, 479.77, 478.63, 477.5, 476.36, 475.22, 474.09, 472.95, 471.81, 470.67, 469.54, 468.4, 467.26, 466.13, 464.99, 463.85, 462.71, 461.58, 460.44, 459.3, 458.17, 457.03, 455.89, 454.76, 453.62, 452.48, 451.34, 450.21, 449.07, 447.93, 446.8, 445.66, 444.52, 443.38, 442.25, 441.11, 439.97, 438.84, 437.7, 436.56, 435.43, 434.29, 433.15, 432.01, 430.88, 429.74, 428.6, 427.47, 426.33, 425.19, 424.05, 422.92, 421.78, 420.64, 419.51, 418.37, 417.23, 416.1, 414.96, 413.82, 412.68, 411.55, 410.41, 409.27, 408.14, 407, 405.86, 404.72, 403.59, 402.45, 401.31, 400.18, 399.04, 397.9, 396.77, 395.63, 394.49, 393.35, 392.22, 391.08, 389.94, 388.81, 387.67, 386.53, 385.4, 384.26, 383.12, 381.98, 380.85, 379.71, 378.57, 377.44, 376.3, 375.16, 374.02, 372.89, 371.75, 370.61, 369.48, 368.34, 367.2, 366.07, 364.93, 363.79, 362.65, 361.52, 360.38, 359.24, 358.11, 356.97, 355.83, 354.69, 353.56, 352.42, 351.28, 350.15, 349.01, 347.87, 346.74, 345.6, 344.46, 343.32, 342.19, 341.05, 339.91, 338.78, 337.64, 336.5, 335.36, 334.23, 333.09, 331.95, 330.82, 329.68, 328.54, 327.41, 326.27, 325.13, 323.99, 322.86, 321.72, 320.58, 319.45, 318.31, 317.17, 316.03, 314.9, 313.76, 312.62, 311.49, 310.35, 309.21, 308.08, 306.94, 305.8, 304.66, 303.53, 302.39, 301.25, 300.12, 298.98, 297.84, 296.7, 295.57, 294.43, 293.29, 292.16, 291.02, 289.88, 288.75, 287.61, 286.47, 285.33, 284.2, 283.06, 281.92, 280.79, 279.65, 278.51, 277.37, 276.24, 275.1, 273.96, 272.83, 271.69, 270.55, 269.42, 268.28, 267.14, 266, 264.87, 263.73, 262.59, 261.46, 260.32, 259.18, 258.04, 256.91, 255.77, 254.63, 253.5, 252.36, 251.22, 250.09, 248.95, 247.81, 246.67, 245.54, 244.4, 243.26, 242.13, 240.99, 239.85, 238.72, 237.58, 236.44, 235.3, 234.17, 233.03, 231.89, 230.76, 229.62, 228.48, 227.34, 226.21, 225.07, 223.93, 222.8, 221.66, 220.52, 219.39, 218.25, 217.11, 215.97, 214.84, 213.7, 212.56, 211.43, 210.29, 209.15, 208.01, 206.88, 205.74, 204.6, 203.47, 202.33, 201.19, 200.06, 198.92, 197.78, 196.64, 195.51, 194.37, 193.23, 192.1, 190.96, 189.82, 188.68, 187.55, 186.41, 185.27, 184.14, 183, 181.86, 180.73, 179.59, 178.45, 177.31, 176.18, 175.04, 173.9, 172.77, 171.63, 170.49, 169.35, 168.22, 167.08, 165.94, 164.81, 163.67, 162.53, 161.4, 160.26, 159.12, 157.98, 156.85, 155.71, 154.57, 153.44, 152.3, 151.16, 150.02, 148.89, 147.75, 146.61, 145.48, 144.34, 143.2, 142.07, 140.93, 139.79, 138.65, 137.52, 136.38, 135.24, 134.11, 132.97, 131.83, 130.7, 129.56, 128.99, 128.42, 127.85, 127.28, 126.72, 126.15, 125.58, 125.01, 124.44, 123.87, 123.3, 122.74, 122.17, 121.6, 121.03, 120.46, 119.89, 119.32, 118.76, 118.19, 117.62, 117.05, 116.48, 115.91, 115.34, 114.78, 114.21, 113.64, 113.07, 112.5, 111.93, 111.37, 110.8, 110.23, 109.66, 109.09, 108.52, 107.95, 107.39, 106.82, 106.25, 105.68, 105.11, 104.54, 103.97, 103.41, 102.84, 102.27, 101.7, 101.13, 100.56, 99.99, 99.43, 98.86, 98.29, 97.72, 97.15, 96.58, 96.01, 95.45, 94.88, 94.31, 93.74, 93.17, 92.6, 92.04, 91.47, 90.9, 90.33, 89.76, 89.19, 88.62, 88.06, 87.49, 86.92, 86.35, 85.78, 85.21, 84.64, 84.08, 83.51, 82.94, 82.37, 81.8, 81.23, 80.66, 80.1, 79.53, 78.96, 78.39, 77.82, 77.25, 76.69, 76.12, 75.55, 74.98, 74.41, 73.84, 73.27, 72.71, 72.14, 71.57, 71, 70.43, 69.86, 69.29, 68.73, 68.16, 67.59, 67.02, 66.45, 65.88, 65.31, 64.75, 64.18, 63.61, 63.04, 62.47, 61.9, 61.33, 60.77, 60.2, 59.63, 59.06, 58.49, 57.92, 57.36, 56.79, 56.22, 55.65, 55.08, 54.51, 53.94, 53.38, 52.81, 52.24, 51.67, 51.1, 50.53, 49.96, 49.4, 48.83, 48.26, 47.69, 47.12, 46.55, 45.98, 45.42, 44.85, 44.28, 43.71, 43.14, 42.57, 42, 41.44, 40.87, 40.3, 39.73, 39.16, 38.59, 38.03, 37.46, 36.89, 36.32, 35.75, 35.18, 34.61, 34.05, 33.48, 32.91, 32.34, 31.77, 31.2, 30.63, 30.07, 29.5, 28.93, 28.36, 27.79, 27.22, 26.65, 26.09, 25.52, 24.95, 24.38, 23.81, 23.24, 22.67, 22.11, 21.54, 20.97, 20.4, 19.83, 19.26, 18.7, 18.13, 17.56, 16.99, 16.42, 15.85, 15.28, 14.72, 14.15, 13.58, 13.01, 12.44, 11.87, 11.3, 10.74, 10.17, 9.6, 9.03, 8.46, 7.89, 7.32, 6.76, 6.19, 5.62, 5.05, 4.48, 3.91, 3.35, 2.78, 2.21, 1.64, 1.07, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

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

  float g_force_short_sample_ = 0.0, g_force_long_sample_;

  void GetAltimeterData();
  void GetAGL();
  void UpdateFlightState();
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
