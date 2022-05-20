//================================================================//
//参考URL1https://nobita-rx7.hatenablog.com/entry/27644291
//参考URL2https://deviceplus.jp/arduino/entry041/
//参考URL3https://www.pasco.co.jp/recommend/word/word046/
//参考URL4https://stemship.com/arduino-basic-digital-filter/
//================================================================//
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include<Wire.h>

//GPS用のモジュール
TinyGPSPlus gps;

//GPSのシリアル通信
SoftwareSerial mySerial(12, 13); // RX, TX

//モーターピンアサイン
const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
const int CH4 = 7;


// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;
double Calib = -60; //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calibx =10;
double Caliby = 55;
float LatA = 35.7121612, LongA =139.760561;  //目的地Aの緯度経度
float LatR, LongR;

float delta_theta; //GPSと9軸の差分
float threshold = 60; //角度の差分の閾値


//バッファの長さ
#define BUF_LEN 10

//バッファ
int buf[BUF_LEN];
int index = 0;

//フィルター後の値
float filterVal =0;

float deg2rad(float deg) {
        return deg * PI / 180.0;
    }

//Hubenyの式

float calculateDistance(float latitude1, float longitude1, float latitude2, float longitude2) {
    // 先に計算しておいた定数
    float e2 = 0.00669437999019758;   // WGS84における「離心率e」の2乗
    float Rx = 6378137.0;             // WGS84における「赤道半径Rx」
    float m_numer = 6335439.32729246; // WGS84における「子午線曲率半径M」の分子(Rx(1-e^2))

   

    float rad_lat1 = deg2rad(latitude1);
    float rad_lon1 = deg2rad(longitude1);
    float rad_lat2 = deg2rad(latitude2);
    float rad_lon2 = deg2rad(longitude2);

    float dp = rad_lon1 - rad_lon2;       // 2点の緯度差
    float dr = rad_lat1 - rad_lat2;       // 2点の経度差
    float p = (rad_lon1 + rad_lon2) / 2.0;// 2点の平均緯度

    float w = sqrt(1.0 - e2 * pow(sin(p), 2));
    float m = m_numer / pow(w, 3);   // 子午線曲率半径
    float n = Rx / w;                     // 卯酉(ぼうゆう)線曲率半径

    // 2点間の距離(単位m)
    float d = sqrt(pow((m * dp), 2)
                      + pow((n *cos(p) * dr), 2));
    return d;
}



void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  mySerial.begin(9600);
  //BMX055 初期化
  BMX055_Init();

   //バッファの初期化
  for(int i=0; i<BUF_LEN; i++) {
    buf[i] = 0;
  }

  pinMode(ENABLE,OUTPUT);  // 7番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 10番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 11版ピンをOUTPUT指定
  pinMode(CH3,OUTPUT);    // 12番ピンをOUTPUT指定
  pinMode(CH4,OUTPUT);    // 13版ピンをOUTPUT指定

  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable
  
  delay(300);
}


//Medianフィルタ関数
int medianFilter() {
  //ソート用のバッファ
  static int sortBuf[BUF_LEN];

  //ソート用バッファにデータをコピー
  for(int i=0; i<BUF_LEN; i++) {
    sortBuf[i] = buf[i];
  }

  //クイックソートで並べ替える
  qsort(sortBuf, BUF_LEN, sizeof(int), quicksortFunc);

  return sortBuf[(int)BUF_LEN/2];
}

//クイックソート関数
int quicksortFunc(const void *a, const void *b) {
  return *(int *)a - *(int *)b;
}


void loop()
{

  BMX055_Mag();
  float x = atan2(yMag-Caliby,xMag-Calibx)/3.14*180+180; //磁北を0°(or360°)として出力
  x = (x+Calib); //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
  
//  Serial.println(x); //ここで進行方向と並行な向きの矢印が磁北（0°）になっているか確認

  x = x + 7;  //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

  //calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整

  if (x>360) {
    x = x-360;
  }

  else if (x<0){
   x = x+360;
  }

  else {
   x = x;
 }

// バッファに取り込んで、インデックスを更新する。
  buf[index] = x;
  index = (index+1)%BUF_LEN;

//フィルタ後の値を計算

  filterVal = medianFilter();
  x = filterVal;

  //GPS情報が取得されると計算が始まる

  while (mySerial.available() > 0){
 char c = mySerial.read();
 gps.encode(c);
 //GPSが更新されたら、ローバーの緯度経度からローバーと目的地の真北を基準にした角度のずれを計算
 if (gps.location.isUpdated()){
 float LatR = gps.location.lat(); //roverの緯度を計算
 float LongR = gps.location.lng(); //roverの経度を計算
 
 float degRtoA = atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180;
 Serial.print("degRtoA=");
 Serial.print(degRtoA); //rvoverと目的地Aの真北を基準にした角度のずれ
 Serial.println("°");

//Hubenyの式で計算
 float RtoA = calculateDistance(LatA, LongA, LatR, LongR);
 Serial.print("RtoA=");
 Serial.print(RtoA);
 Serial.println("m");
 //BMX055 磁気の読み取り
//  BMX055_Mag();
//  float x = atan2(yMag-Caliby,xMag-Calibx)/3.14*180+180; //磁北を0°(or360°)として出力
//  x = (x+Calib); //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
//  
////  Serial.println(x); //ここで進行方向と並行な向きの矢印が磁北（0°）になっているか確認
//
//  x = x + 7;  //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける
//
//  //calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整
//
//  if (x>360) {
//    x = x-360;
//  }
//
//  else if (x<0){
//   x = x+360;
//  }
//
//  else {
//   x = x;
// }

 //バッファに取り込んで、インデックスを更新する。
//  buf[index] = x;
//  index = (index+1)%BUF_LEN;
//
////フィルタ後の値を計算
//
//  filterVal = medianFilter();
//  x = filterVal;

  //シリアル出力
//  Serial.print("Raw");
//  Serial.print(",");
//  Serial.println("Filtered");
  Serial.print("x=");
  Serial.print(x);
  Serial.println("°");
//  Serial.print(",");
//  Serial.println(filterVal);
 
  //ローバーの進行方向と目的地の角度のずれdelta_thetaを計算
  //判定に用いるdelta_thetaの閾値は任意

  if (x < degRtoA){
    float delta_theta = degRtoA - x;
    Serial.print(delta_theta);
    Serial.println("°");
    if ((0 < delta_theta && delta_theta < threshold/2)|| (360 - threshold/2 < delta_theta && delta_theta < 360)){ //両開き、閾値に入ったとき //閾値内に向いている時
    Serial.println("This is the right direction!");

    //直進
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH1,LOW);    
    digitalWrite(CH2,HIGH); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,LOW);

    delay(500);

    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
  }
  else { //閾値内に向いていない時
    Serial.println("This is the wrong direction!");

    //回転
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH1,HIGH);    
    digitalWrite(CH2,LOW); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,LOW);

    delay(200);

    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
    
  }
  }
else {
    float delta_theta = 360 + degRtoA - x;
    Serial.println(delta_theta);
    if ((0 < delta_theta && delta_theta < threshold/2)|| (360 - threshold/2 < delta_theta && delta_theta < 360)){ //両開き、閾値に入ったとき
    Serial.println("This is the right direction!");

    //直進
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH1,LOW);    
    digitalWrite(CH2,HIGH); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,LOW);

    delay(500);

    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
      
  }
  else {
    Serial.println("This is the wrong direction!");

    //回転
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH1,HIGH);    
    digitalWrite(CH2,LOW); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,LOW);

    delay(200);

    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
      
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
