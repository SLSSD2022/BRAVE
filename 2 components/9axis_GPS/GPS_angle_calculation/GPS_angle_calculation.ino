//================================================================//
//参考URL1https://nobita-rx7.hatenablog.com/entry/27644291
//参考URL2https://deviceplus.jp/arduino/entry041/
//参考URL3https://www.pasco.co.jp/recommend/word/word046/
//================================================================//
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include<Wire.h>

//GPS用のモジュール
TinyGPSPlus gps;

//GPSのシリアル通信
SoftwareSerial mySerial(13, 12); // RX, TX


// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;
double Calib = -35; //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）

float LatA = 35.7100069, LongA = 139.8108103;  //目的地Aの緯度経度(今回はスカイツリー)
int threshold = 30; //角度の差分の閾値

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  mySerial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);
}

void loop()
{
  //Serial.println("--------------------------------------");

  while (mySerial.available() > 0){
    char c = mySerial.read();
    //Serial.print(c);
    gps.encode(c);
    //GPSが更新されたら、ローバーの緯度経度からローバーと目的地の真北を基準にした角度のずれを計算
    if (gps.location.isUpdated()){
      float LatR = gps.location.lat();
      float LongR = gps.location.lng();
      // Serial.print("ROVER'S LAT ="); Serial.println(gps.location.lat(), 6);
      // Serial.print("Rover's LONG="); Serial.println(gps.location.lng(), 6);
      //Serial.print(atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180);
     
      float degRtoA = atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180;
      Serial.println(degRtoA);
      //degRtoAはroverと目的地Aの真北を基準にした角度のずれ
       
      // Serial.print("deg: The distance between Rover and A is= ");    
      // int RtoA = sqrt(pow(LongR - LongA, 2) + pow(LatR - LatA, 2)) * 99096.44;
      // Serial.print(RtoA);
      // Serial.println("m");

      
      //BMX055 磁気の読み取り
      BMX055_Mag();
      float x = atan2(yMag-105,xMag+60)/3.14*180+180; //磁北を0°(or360°)として出力
      x = (x+Calib); //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
      
      //calibを足したことでcalib°~360+calib°で出力されてしまうので、0°~360°になるよう調整
      if (x>360) {
        x = x-360;
      }
      
      else if (x<0){
        x = x+360;
      }
      
      else {
        x = x;
      }
      
      x += 7;
      Serial.println(x);
        
      //delay(100);
    
      //磁北は真北に対して西に（反時計回りに)7°ずれているため、ここで補正をかけつつ、ローバーの進行方向と目的地の角度のずれdelta_thetaを計算
      //判定に用いるdelta_thetaの閾値は任意
    
      float delta_theta;
    
      
      if (x < degRtoA){
        delta_theta = degRtoA - x;
        Serial.print("x < degRtoA:");
        Serial.print(delta_theta);
        Serial.print(":");
        
        //閾値内にあるときは真っ直ぐ
        if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= degRtoA)){
          Serial.println("This is the right direction!");
        }
        else {
          Serial.println("This is the wrong direction!");
        }
      }
    
      
      else{
        delta_theta = x - degRtoA;
        Serial.print("degRtoA < x:");
        Serial.print(delta_theta);
        Serial.print(":");
       
        //閾値内にあるときは真っ直ぐ
        if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= 360)){
          Serial.println("This is the right direction!");
        }
    
        //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
        else {
          Serial.println("This is the wrong direction!");
        }
      }
    }
  }
}

//=====================================================================================//
void BMX055_Init()
{

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
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
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
