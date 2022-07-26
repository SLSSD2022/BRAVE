#include "./IMU.h"
#include <Wire.h>
//===========9axis sensor function==========================================================================//
IMU::IMU()
  :xGyro(0)
  ,yGyro(0)
  ,zGyro(0)
  ,xMag(0)
  ,yMag(0)
  ,zMag(0)
  ,calib(-5)
  ,calibx(20)
  ,caliby(132)
{
}

IMU::IMU(float xG,float yG,float zG,int xM,int yM,int zM,float ca,float cax,float cay)
  :xGyro(xG)
  ,yGyro(yG)
  ,zGyro(zG)
  ,xMag(xM)
  ,yMag(yM)
  ,zMag(zM)
  ,calib(ca)
  ,calibx(cax)
  ,caliby(cay)
{
}

void IMU::init()
{
  
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F); // Select Range register

  //ここでWire.write()の中の値を変えることでスケール変更可能
  // 0x00, 2000deg/s
  // 0x01, 1000deg/s
  // 0x02, 500deg/s
  // 0x03, 250deg/s
  // 0x04, 125deg/s

  Wire.write(0x04); // Full scale = +/- 500 degree/s

  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10); // Select Bandwidth register
  Wire.write(0x07); // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11); // Select LPM1 register
  Wire.write(0x00); // Normal mode, sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x83); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x01); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C); // Select Mag register
  Wire.write(0x38); // Normal Mode, ODR = 10 Hz(0x00), ODR = 20Hz(0x28), ODR = 30Hz(0x38)
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x17); // No. of Repetitions for X-Y Axis: Normal 9(0x04), Expand 15(0x07), Max 47(0x17)
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x52); // No. of Repetitions for Z-Axis: Normalmiss? 23(0x16), Normal 16(0x0E), Expand 27(0x1A), Max 83(0x52)
  Wire.endTransmission();
  Serial.println("IMU module initialized!");
}


void IMU::getGyro()
{
  unsigned int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767){
    xGyro -= 65536;
  }
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767){
    yGyro -= 65536;
  }
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767){
    zGyro -= 65536;
  }

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}


void IMU::getMag()
{
  unsigned int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] << 5) | (data[0] >> 3));
  if (xMag > 4095){
    xMag -= 8192;
  }
  yMag = ((data[3] << 5) | (data[2] >> 3));
  if (yMag > 4095){
    yMag -= 8192;
  }
  zMag = ((data[5] << 7) | (data[4] >> 1));
  if (zMag > 16383){
    zMag -= 32768;
  }
}
//
//boolean IMU::calibration()
//{
//  bufx[calIndex] = imu.xMag;
//  bufy[calIndex] = imu.yMag;
//  if (calIndex == CAL_BUF_LEN - 1) { //バッファに値がたまったら
//    imu.calibx = xcenter_calculation();
//    imu.caliby = ycenter_calculation();
//    rover.status.calibration = 0 ;
//    Serial.print(":calib_x:");
//    Serial.print(imu.calibx);
//    Serial.print(":calib_y:");
//    Serial.print(imu.caliby);
//    calIndex = 0;
//    return true;
//  }
//  else {
//    calIndex = (calIndex + 1) % CAL_BUF_LEN;
//    return false;
//  }
//}

void IMU::getAngle()
{
  this->x = atan2(this->yMag - this->caliby, this->xMag - this->calibx) / 3.14 * 180 + 180; //磁北を0°(or360°)として出力
  x += this->calib;
  x -= 7; //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

  // calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整

  if (x > 360)
  {
    x -= 360;
  }

  else if (x < 0)
  {
    x += 360;
  }
}

int IMU::angleCalculation()
{
  this->getAngle();

  // バッファに取り込んで、インデックスを更新する。
  this->buf[this->index] = this->x;
  this->index = (this->index + 1) % BUF_LEN;
  //フィルタ後の値を計算
  int filterVal = this->medianFilter();
  return filterVal;
}

// Medianフィルタ関数
int IMU::medianFilter()
{
  //ソート用のバッファ
  static int sortBuf[BUF_LEN];

  //ソート用バッファにデータをコピー
  for (int i = 0; i < BUF_LEN; i++)
  {
    sortBuf[i] = this->buf[i];
  }

  //クイックソートで並べ替える
  qsort(sortBuf, BUF_LEN, sizeof(int), quicksortFunc);

  return sortBuf[(int)BUF_LEN / 2];
}


//クイックソート関数
int quicksortFunc(const void *a, const void *b)
{
  return *(int *)a - *(int *)b;
}


void IMU::printAll()
{ 
  Serial.print(":xMag:");
  Serial.print(this->xMag);
  Serial.print(":yMag:");
  Serial.print(this->yMag);
  Serial.print(":calibx:");
  Serial.print(this->calibx);
  Serial.print(":caliby:");
  Serial.print(this->caliby);
  Serial.print(":x:");
  Serial.print(this->x);
}
