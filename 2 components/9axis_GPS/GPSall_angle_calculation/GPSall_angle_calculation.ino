#include <TinyGPSPlus.h>
#include<Wire.h>
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

TinyGPSPlus gps;
boolean GPS_flag = 1;

// センサーの値を保存するグローバル変数
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;


double Calib = 180; //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calibx = 28;
double Caliby = 143;
float LatA = 35.7100069, LongA = 139.8108103;  //目的地Aの緯度経度(今回はスカイツリー)
float LatR, LongR;

float degRtoA; //目的地とGPS現在地の角度
float delta_theta;
int threshold = 30; //角度の差分の閾値


//バッファの長さ
#define BUF_LEN 10

//バッファ
int buf[BUF_LEN];
int index = 0;

//int buf_degRtoA[BUF_LEN];
//int index_degRtoA = 0;

//フィルター後の値
float filterVal =0;

float deg2rad(float deg) {
        return deg * PI / 180.0;
    }


void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  Serial1.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(500);
}


void loop()
{

//  //BMX055 ジャイロの読み取り
//  BMX055_Gyro();
  //BMX055 磁気の読み取り
  BMX055_Mag();
  float x = atan2(yMag-Caliby,xMag-Calibx)/3.14*180+180; //磁北を0°(or360°)として出力
  x = (x+Calib);
  x = x - 7;  //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

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


  //---------------------GPS取得--------------------------------------------------
  while (Serial1.available() > 0 && GPS_flag == 1){
//    Serial.print("YES");
    char c = Serial1.read();
//    Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated()){
      Serial.println("");
      Serial.println("I got new GPS!");
      LatR = gps.location.lat(); //roverの緯度を計算
      LongR = gps.location.lng(); //roverの経度を計算
      degRtoA = atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180;
      GPS_flag = 0;
    }
    //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
    delay(1);
  }
  GPS_flag =1;
  //---------------------GPS取得--------------------------------------------------
  //バッファから取り出す(一番最後に取得したGPSを取り出す)
//  degRtoA = buf_degRtoA[index_degRtoA];
  Serial.print("degRtoA:");Serial.print(degRtoA);
  
  Serial.print(":x:");Serial.print(x); 

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




//=====================================================================================//
void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register

  //ここでWire.write()の中の値を変えることでスケール変更可能
  //0x00, 2000deg/s
  //0x01, 1000deg/s
  //0x02, 500deg/s
  //0x03, 250deg/s
  //0x04, 125deg/s
  
  Wire.write(0x04);  // Full scale = +/- 500 degree/s

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
