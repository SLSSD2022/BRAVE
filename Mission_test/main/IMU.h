#ifndef _IMU_HEAD_
#define _IMU_HEAD_

//------------------------------9axis sensor------------------------------
// I2C address for BMX055 Gyro
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// I2C address for BMX055 Magnetic
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)
#define BUF_LEN 10
//#define CAL_BUF_LEN 100


class IMU {
public:
  float xGyro;
  float yGyro;
  float zGyro;
  int xMag;
  int yMag;
  int zMag;
  int x;

  //Constant for calibration(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
  float calib;
  float calibx;
  float caliby;

  IMU();
  IMU(float xG,float yG,float zG,int xM,int yM,int zM,float ca,float cax,float cay) ;
  void init();
  void getGyro();
  void getMag();
//  boolean calibration();
  void getAngle();
  int angleCalculation();
  void printAll();
private: 
  int buf[BUF_LEN];
  int index;
//  //キャリブレーション用バッファの長さ
//  int bufx[CAL_BUF_LEN];
//  int bufy[CAL_BUF_LEN];
//  int calIndex = 0;
  
  int medianFilter();
};

int quicksortFunc(const void *a, const void *b);

#endif
