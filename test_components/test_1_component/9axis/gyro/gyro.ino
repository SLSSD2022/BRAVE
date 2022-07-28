//================================================================//
//  AE-BMX055             Arduino UNO                             //
//    VCC                    +5V                                  //
//    GND                    GND                                  //
//    SDA                    A4(SDA)                              //
//    SCL                    A5(SCL)                              //
//                                                                //
//   (JP4,JP5,JP6はショートした状態)                                //
//   http://akizukidenshi.com/catalog/g/gK-13010/                 //
//================================================================//

#include<Wire.h>
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
float zGyrobias = 5.10/58.438;
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;
int calibx = -30;
int caliby = 73;
int calib = 316;
unsigned long stopi,starti;
float degGyro= 0;
float degMag= 0;

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);
  
  //BMX055 磁気の読み取り
  BMX055_Mag();
//  Serial.print(xMag);
//  Serial.print(",");
//  Serial.print(yMag);
//  Serial.print(",");
  degMag = atan2(yMag - caliby, xMag - calibx) / 3.14 * 180. + 180.; //磁北を0°(or360°)として出力
  degMag += calib;
  degMag -= 7; //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

  // calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整
  if (degMag > 360.)
  {
    degMag -= 360.;
  }
  if (degMag < 0)
  {
    degMag += 360.;
  }

  degGyro = degMag;
  
  starti = millis();
}

void loop()
{
  
  //BMX055 ジャイロの読み取り
  BMX055_Gyro();
  zGyro -= zGyrobias;
  stopi = millis();
//  Serial.print(zGyro);
//  Serial.print(",");
//  Serial.print(stopi -starti);
//  Serial.print(",");
//  Serial.print(zGyro*(stopi -starti)/1000);
  degGyro = degGyro - zGyro*(stopi -starti)/1000;
  starti = millis();
  if (degGyro > 360.)
  {
    degGyro -= 360.;
  }
  if (degGyro < 0)
  {
    degGyro += 360.;
  }
  Serial.print(degGyro);
  Serial.print(",");
  
  //BMX055 磁気の読み取り
  BMX055_Mag();
//  Serial.print(xMag);
//  Serial.print(",");
//  Serial.print(yMag);
//  Serial.print(",");
  degMag = atan2(yMag - caliby, xMag - calibx) / 3.14 * 180. + 180.; //磁北を0°(or360°)として出力
  degMag += calib;
  degMag -= 7; //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

  // calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整
  if (degMag > 360.)
  {
    degMag -= 360.;
  }
  if (degMag < 0)
  {
    degMag += 360.;
  }
  Serial.print(degMag);
  
  
  delay(20);
  Serial.println(""); 
}

//=====================================================================================//
void BMX055_Init()
{
  
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x38); // Normal Mode, ODR = 10 Hz(0x00), ODR = 20Hz(0x28), ODR = 30Hz(0x38)
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x17); // No. of Repetitions for X-Y Axis: Normal 9(0x04), Expand 15(0x07), Max 47(0x17)
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x52); // No. of Repetitions for Z-Axis: Normalmiss? 23(0x16), Normal 16(0x0E), Expand 27(0x1A), Max 83(0x52)
  Wire.endTransmission();
}
//=====================================================================================//
//void BMX055_Accl()
//{
//  unsigned int data[6];
//  for (int i = 0; i < 6; i++)
//  {
//    Wire.beginTransmission(Addr_Accl);
//    Wire.write((2 + i));// Select data register
//    Wire.endTransmission();
//    Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
//    // Read 6 bytes of data
//    // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
//    if (Wire.available() == 1)
//      data[i] = Wire.read();
//  }
//  // Convert the data to 12-bits
//  xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
//  if (xAccl > 2047)  xAccl -= 4096;
//  yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
//  if (yAccl > 2047)  yAccl -= 4096;
//  zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
//  if (zAccl > 2047)  zAccl -= 4096;
//  xAccl = xAccl * 0.0098; // range = +/-2g
//  yAccl = yAccl * 0.0098; // range = +/-2g
//  zAccl = zAccl * 0.0098; // range = +/-2g
//}
//=====================================================================================//
void BMX055_Gyro()
{
  unsigned int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)  xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)  yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)  zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}
//=====================================================================================//
void BMX055_Mag()
{
  unsigned int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
// Convert the data
  xMag = ((data[1] <<5) | (data[0]>>3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] <<5) | (data[2]>>3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] <<7) | (data[4]>>1));
  if (zMag > 16383)  zMag -= 32768;
}