#include <TinyGPSPlus.h>

TinyGPSPlus gps;
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

//定数群
//a, F: 世界測地系-測地基準系1980（GRS80）楕円体
double m0 = 0.9999;
long a = 6378137;
double F = 298.257222101;
//n, A_i, alpha_iの計算
double n = 0.00167922039462874;
double A_0 = 1.00000070494541;
double A_1 = -0.00251882970412393;
double A_2 = 2.6435429493241E-06;
double A_3 = -3.45262590730741E-09;
double A_4 = 4.89183042438795E-12;
double A_5 = -7.22872604581392E-15;
double a1 = 0.000837731824728547;
double a2 = 7.60852784837925E-07;
double a3 = 1.19764550023156E-09;
double a4 = 2.42915026065425E-12;
double a5 = 5.75016438409197E-15;
double A_ = 6366812.40085647;
//基準点に依存する定数
double phi0_deg;
double lambda0_deg;
double phi0_rad;
double lambda0_rad;
double mid_S_;
double S_;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
//
//  calc_const(35.71556091,139.76123046875);
//  double x = 0;
//  double y = 0;
//  calc_xy(35.7155075073242, 139.761245727539, &x, &y);
//  Serial.print(":x:");
//  Serial.print(x,10);
//  Serial.print(":y:");
//  Serial.println(y,10);
  delay(1000);
  
  Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
}

void loop() { // run over and over
  double x = 0;
  double y = 0;


  

  while (Serial1.available() > 0) {
    char c = Serial1.read();
//    Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated()) {
//      Serial.println("YEAHHHHHHHHHHHHHHHHHHHHH!");
//       Serial.print("LAT=");
      Serial.print(gps.location.lat(), 13);
//       Serial.print("LONG=");
      Serial.print(",");
      Serial.print(gps.location.lng(), 13);
//      Serial.print(",");
//      Serial.print(gps.location.lat() - 35.7199478149414, 13);
////       Serial.print("LONG=");
//      Serial.print(",");
//      Serial.print(gps.location.lng() - 139.735931396484, 13);
//      float RtoA = sqrt(pow(gps.location.lng() - 139.735931396484, 2) + pow(gps.location.lat() - 35.7199478149414, 2)) * 99096.44;
//      Serial.print(",");
//      Serial.print(RtoA, 2);
//      if(phi0_deg == 0){
//        calc_const(gps.location.lat(),gps.location.lng());
//      }
//      calc_xy(gps.location.lat(),gps.location.lng(), &x, &y);
      
//      Serial.print(",");
//      Serial.print(x);
//      Serial.print(",");
//      Serial.print(y);
      Serial.println("");
    }
  }
}


double deg2rad(double deg)
{
  return (double)(deg * PI / 180.0);
}

double atanh(double x)
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

int calc_const(double phi_deg, double lambda_deg){
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


void calc_xy(double phi_deg, double lambda_deg ,double* x,double* y){
//緯度経度を平面直角座標に変換する
//- input:
//(phi_deg, lambda_deg): 変換したい緯度・経度[度]（分・秒でなく小数であることに注意）
//(phi0_deg, lambda0_deg): 平面直角座標系原点の緯度・経度[度]（分・秒でなく小数であることに注意）
//- output:
//x: 変換後の平面直角座標[m]
//y: 変換後の平面直角座標[m]
  
//緯度経度・平面直角座標系原点をラジアンに直す
  double phi_rad = deg2rad(phi_deg);
  double lambda_rad = deg2rad(lambda_deg);
//  Serial.print(":phi_rad:");
//  Serial.print(phi_rad,10);
//  Serial.print(":lambda_rad:");
//  Serial.print(lambda_rad,10);
//  
//lambda_c, lambda_sの計算
  double lambda_c = cos(lambda_rad - lambda0_rad);
  double lambda_s = sin(lambda_rad - lambda0_rad);
//  Serial.print(":lamda_c:");
//  Serial.print(lambda_c,10);
//  Serial.print(":lamda_s:");
//  Serial.print(lambda_s,10);

//t, t_の計算
  double t = sinh(atanh(sin(phi_rad)) - (2*sqrt(n) / (1+n))*atanh(2*sqrt(n) / (1+n) * sin(phi_rad)) );
//  double t_o = atanh(sin(phi_rad)) - (2*sqrt(n) / (1+n))*atanh(2*sqrt(n) / (1+n) * sin(phi_rad)) ;
  double t_ = sqrt(1 + t*t);
//  Serial.print(":t:");
//  Serial.print(t,10);
//  Serial.print(":t_:");
//  Serial.print(t_,10);

//xi', eta'の計算
  double xi2  = atan(t / lambda_c); // [rad]
  double eta2 = atanh(lambda_s / t_);
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
