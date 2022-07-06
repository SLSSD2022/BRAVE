#include "./IMU.h"

IMU imu;
void setup()
{
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  imu.init();
  delay(300);
}

void loop()
{
  imu.getGyro();
  imu.getMag();
  imu.getAngle();
  imu.printAll();
  Serial.println("");
  delay(1000);
}
