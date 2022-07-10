#include <Tinygps++.h>
#include <HardwareSerial.h>
#include "./GPS.h"

//静的メンバ変数の再宣言
//a, F: 世界測地系-測地基準系1980（GRS80）楕円体
float GPS::m0 = 0.9999;
long GPS::a = 6378137;
float GPS::F = 298.257222101;
//n, A_i, alpha_iの計算
float GPS::n = 0.00167922039462874;
float GPS::A_0 = 1.00000070494541;
float GPS::A_1 = -0.00251882970412393;
float GPS::A_2 = 2.6435429493241E-06;
float GPS::A_3 = -3.45262590730741E-09;
float GPS::A_4 = 4.89183042438795E-12;
float GPS::A_5 = -7.22872604581392E-15;
float GPS::a1 = 0.000837731824728547;
float GPS::a2 = 7.60852784837925E-07;
float GPS::a3 = 1.19764550023156E-09;
float GPS::a4 = 2.42915026065425E-12;
float GPS::a5 = 5.75016438409197E-15;
float GPS::A_ = 6366812.40085647;

//=========GPS and position function============================================================================//
GPS::GPS()
    :HWSerial(&Serial1)
    ,phi0_deg(0)
    ,lambda0_deg(0)
    ,phi0_rad(0)
    ,lambda0_rad(0)
    ,mid_S_(0)
    ,S_(0)
{
}

GPS::GPS(HardwareSerial *serialport)
    :HWSerial(serialport)
    ,phi0_deg(0)
    ,lambda0_deg(0)
    ,phi0_rad(0)
    ,lambda0_rad(0)
    ,mid_S_(0)
    ,S_(0)
{
}

void GPS::init(){
    HWSerial->begin(9600);
    while (!HWSerial) {
        // wait for serial port to connect. Needed for native USB port only
    }
    Serial.println("GPS module initialized!");
}

void GPS::updateGPSlocation(float* lat,float* lng) {
//  int start = millis();
  int updateFlag = 0;
  while(updateFlag == 0){
//    Serial.println("try to catch GPS...");
    while (HWSerial->available() > 0)
    {
      //    Serial.print("YES");
      char c = HWSerial->read();
//      Serial.print(c);
      this->encode(c);
      if (this->location.isUpdated())
      {
        Serial.println("");
        Serial.println("I got new GPS!");
        *lat = this->location.lat();  // roverの緯度を計算
        *lng = this->location.lng(); // roverの経度を計算
        updateFlag = 1;
        break;
      }
      //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
      delay(1);
    }
//    int stop = millis();
//    if(stop > (start - 1000)){
//      Serial.println("cannot catch GPS...");
//      break;
//    }
  }
  
}

float GPS::deg2rad(float deg)
{
  return (float)(deg * PI / 180.0);
}

float GPS::atanh(float x)
{
//  Serial.println(x);
  if(-1. < x && x < 1.){
//    Serial.println("ANSWER!");
//    Serial.println(log((1+x)/(1-x))/2);
    return log((1+x)/(1-x))/2;
  }
  else{
    Serial.println("ERROR!:atanh needs x to be -1<x<1");
    return 0;
  }
} 


int GPS::calc_const(float phi_deg, float lambda_deg){
  //基準点に依存する定数
  phi0_deg = phi_deg;
  lambda0_deg = lambda_deg;
  phi0_rad = deg2rad(phi0_deg);
  lambda0_rad = deg2rad(lambda0_deg);
  mid_S_ = sin(2*phi0_rad)*A_1 + sin(2*phi0_rad*2)*A_2 + sin(2*phi0_rad*3)*A_3 + sin(2*phi0_rad*4)*A_4 + sin(2*phi0_rad*5)*A_5;
  S_ = m0*a/(1+n)*(A_0*phi0_rad + mid_S_);
//  Serial.print(":mid_S_:");
//  Serial.print(mid_S_,10);
//  Serial.print(":S_:");
//  Serial.print(S_,10);
}


void GPS::calc_xy(float phi_deg, float lambda_deg ,float* x,float* y){
//緯度経度を平面直角座標に変換する
//- input:
//(phi_deg, lambda_deg): 変換したい緯度・経度[度]（分・秒でなく小数であることに注意）
//(phi0_deg, lambda0_deg): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
//- output:
//x: 変換後の平面直角座標[m]
//y: 変換後の平面直角座標[m]
  
//緯度経度・平面直角座標系原点をラジアンに直す
  float phi_rad = deg2rad(phi_deg);
  float lambda_rad = deg2rad(lambda_deg);
//  Serial.print(":phi_rad:");
//  Serial.print(phi_rad,10);
//  Serial.print(":lambda_rad:");
//  Serial.print(lambda_rad,10);
//  
//lambda_c, lambda_sの計算
  float lambda_c = cos(lambda_rad - lambda0_rad);
  float lambda_s = sin(lambda_rad - lambda0_rad);
//  Serial.print(":lamda_c:");
//  Serial.print(lambda_c,10);
//  Serial.print(":lamda_s:");
//  Serial.print(lambda_s,10);

//t, t_の計算
  float t = sinh(atanh(sin(phi_rad)) - (2*sqrt(n) / (1+n))*atanh(2*sqrt(n) / (1+n) * sin(phi_rad)) );
//  float t_o = atanh(sin(phi_rad)) - (2*sqrt(n) / (1+n))*atanh(2*sqrt(n) / (1+n) * sin(phi_rad)) ;
  float t_ = sqrt(1 + t*t);
//  Serial.print(":t:");
//  Serial.print(t,10);
//  Serial.print(":t_:");
//  Serial.print(t_,10);

//xi', eta'の計算
  float xi2  = atan(t / lambda_c); // [rad]
  float eta2 = atanh(lambda_s / t_);
//  Serial.print(":xi2:");
//  Serial.print(xi2,10);
//  Serial.print(":eta2:");
//  Serial.print(eta2,10);

//x, yの計算
  *x = A_ * (xi2 + a1*sin(2*xi2)*cosh(2*eta2) + a2*sin(2*xi2*2)*cosh(2*eta2*2) + a3*sin(2*xi2*3)*cosh(2*eta2*3) + a4*sin(2*xi2*4)*cosh(2*eta2*4) + a5*sin(2*xi2*5)*cosh(2*eta2*5)) - S_; //[m]
//  Serial.print(":x_pre:");
//  Serial.print(A_ * (xi2 + a1*sin(2*xi2)*cosh(2*eta2) + a2*sin(2*xi2*2)*cosh(2*eta2*2) + a3*sin(2*xi2*3)*cosh(2*eta2*3) + a4*sin(2*xi2*4)*cosh(2*eta2*4) + a5*sin(2*xi2*5)*cosh(2*eta2*5)));
  *y = A_ * (eta2 + a1*cos(2*xi2)*sinh(2*eta2) + a2*cos(2*xi2*2)*sinh(2*eta2*2) + a3*cos(2*xi2*3)*sinh(2*eta2*3) + a4*cos(2*xi2*4)*sinh(2*eta2*4) + a5*cos(2*xi2*5)*sinh(2*eta2*5)); // [m]
}
