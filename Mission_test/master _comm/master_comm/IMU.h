#ifndef _IMU_HEAD_
#define _IMU_HEAD_

//------------------------------9axis sensor------------------------------
// I2C address for BMX055 Gyro
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// I2C address for BMX055 Magnetic
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)

typedef struct _bmx055 {
  float xGyro;
  float yGyro;
  float zGyro;
  int xMag;
  int yMag;
  int zMag;

  //Constant for calibration(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
  float calib;
  float calibx;
  float caliby;

  void init();
  void getGyro();
  void getMag();
} bmx055;

bmx055 imu = {0.00, 0.00, 0.00, 0, 0, 0, 175, 20, 132};

#endif
