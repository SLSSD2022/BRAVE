#include "./IMU.h"
#include <Wire.h>

IMU imu;
void setup()
{
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  Wire.begin();
  imu.init();
  delay(300);
}

void loop()
{
  imu.getGyro();
  imu.getMag();
  imu.getAngle();
  int x_ = imu.angleCalculation();
  imu.printAll();
  Serial.print(":x_filtered:");
  Serial.print(x_);
  Serial.println("");
  delay(100);
}
