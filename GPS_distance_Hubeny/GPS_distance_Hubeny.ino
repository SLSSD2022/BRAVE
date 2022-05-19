//================================================================//
//参考URL1https://komoriss.com/calculate-distance-between-two-points-from-latitude-and-longitude/
//参考コードhttps://gist.github.com/hirohitokato/03e98332b10a9ff211e2d9b8d9c3d4fe
//================================================================//
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include<Wire.h>

//GPS用のモジュール
TinyGPSPlus gps;

//GPSのシリアル通信
SoftwareSerial mySerial(12, 13); // RX, TX

float LatA = 35.71539304, LongA =139.7612181;  //目的地Aの緯度経度

float LatR, LongR;

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
  delay(300);
}

void loop()
{
  
  //GPS情報が取得されると計算が始まる

  while (mySerial.available() > 0){
 char c = mySerial.read();
 gps.encode(c);
 if (gps.location.isUpdated()){
 float LatR = gps.location.lat(); //roverの緯度を計算
 float LongR = gps.location.lng(); //roverの経度を計算

// float delta_Long = LongR - LongA;
 
 float RtoA = calculateDistance(LatA, LongA, LatR, LongR);
// float degRtoA = 90 - atan2(sin(delta_Long),cos(LatA)*tan(LatR) - sin(LatA)*cos(delta_Long));
 
 Serial.println(RtoA);
// Serial.println(degRtoA);
 
// Serial.println("m");

 }
  }
}
