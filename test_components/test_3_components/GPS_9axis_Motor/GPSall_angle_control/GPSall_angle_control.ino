#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)

TinyGPSPlus gps;

// GPSのシリアル通信
SoftwareSerial mySerial(12, 13); // RX, TX

//モーター
const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
const int CH4 = 7;

// センサーの値を保存するグローバル変数
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;

double Calib = -60; //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calibx = 10;
double Caliby = 55;
double LatA = 35.7121612, LongA = 139.760561; //目的地Aの緯度経度
double LatR, LongR;

float degRtoA;      //目的地とGPS現在地の角度
int threshold = 30; //角度の差分の閾値

float delta_theta;
float x;

boolean GPS_flag = 1;

int Normal_speed = 100;
int speed_R;
int speed_L;

//バッファの長さ
#define BUF_LEN 10

//バッファ
int buf[BUF_LEN];
int index = 0;

int buf_degRtoA[BUF_LEN];
int index_degRtoA = 0;

//フィルター後の値
float filterVal = 0.0;

float deg2rad(float deg)
{
  return (float) (deg * PI / 180.0);
}

double deg2rad(double deg)
{
  return (double) (deg * PI / 180.0);
}

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  mySerial.begin(9600);
  // BMX055 初期化
  BMX055_Init();

  pinMode(CH1, OUTPUT);
  pinMode(CH2, OUTPUT);
  pinMode(CH3, OUTPUT);
  pinMode(CH4, OUTPUT);
  digitalWrite(ENABLE, LOW); // disable
  delay(500);
}

void loop()
{

  // BMX055 ジャイロの読み取り
  BMX055_Gyro();
  // BMX055 磁気の読み取り
  BMX055_Mag();

  //磁北を0°(or360°)として出力
  //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける
  // calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整
  x = atan2(yMag - Caliby, xMag - Calibx) / 3.14 * 180 + 180 + Calib + 7;
  if (x > 360)
  {
    x = x - 360;
  }
  else if (x < 0)
  {
    x = x + 360;
  }

  // バッファに取り込んで、インデックスを更新する。
  buf[index] = x;
  index = (index + 1) % BUF_LEN;
  //フィルタ後の値を計算
  filterVal = medianFilter();
  x = filterVal;

  //---------------------GPS取得--------------------------------------------------
  if (mySerial.available() > 0)
  {
    char c = mySerial.read();
    gps.encode(c);
    if (gps.location.isUpdated())
    {
      float LatR = gps.location.lat();  // roverの緯度を計算
      float LongR = gps.location.lng(); // roverの経度を計算
      degRtoA = atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180;
      // float RtoA = calculateDistance(LatA, LongA, LatR, LongR);
      //  バッファに取り込んで、インデックスを更新する。
      index_degRtoA = (index_degRtoA + 1) % BUF_LEN;
      buf_degRtoA[index_degRtoA] = degRtoA;
    }
  }
  //---------------------GPS取得--------------------------------------------------
  //バッファから取り出す(一番最後に取得したGPSを取り出す)
  degRtoA = buf_degRtoA[index_degRtoA];
  Serial.print("degRtoA:");
  Serial.print(degRtoA);
  Serial.print(":");

  Serial.print("axis:");
  Serial.print(x);
  Serial.print(":");

  if (x < degRtoA)
  {
    delta_theta = degRtoA - x;
    Serial.print("x < degRtoA:");
    Serial.print(delta_theta);
    Serial.print(":");

    //閾値内にあるときは真っ直ぐ
    if ((0 <= delta_theta && delta_theta <= threshold / 2) || (360 - threshold / 2 <= delta_theta && delta_theta <= degRtoA))
    {
      speed_R = Normal_speed;
      speed_L = Normal_speed;
      analogWrite(CH1, speed_R);
      analogWrite(CH3, speed_L);
      Serial.print("Go straight");
    }
    //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else if (threshold / 2 < delta_theta && delta_theta <= 180)
    {
      speed_R = Normal_speed;
      speed_L = Normal_speed - (delta_theta * Normal_speed / 180);
      analogWrite(CH1, speed_R);
      analogWrite(CH3, speed_L);
      Serial.print("turn left");
    }

    //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else
    {
      speed_R = Normal_speed - ((360 - delta_theta) * Normal_speed / 180);
      speed_L = Normal_speed;
      analogWrite(CH1, speed_R);
      analogWrite(CH3, speed_L);
      Serial.print("turn right");
    }
  }

  else
  {
    delta_theta = x - degRtoA;
    Serial.print("degRtoA < x:");
    Serial.print(delta_theta);
    Serial.print(":");

    //閾値内にあるときは真っ直ぐ
    if ((0 <= delta_theta && delta_theta <= threshold / 2) || (360 - threshold / 2 <= delta_theta && delta_theta <= 360))
    {
      speed_R = Normal_speed;
      speed_L = Normal_speed;
      analogWrite(CH1, speed_R);
      analogWrite(CH3, speed_L);
      Serial.print("Go straight");
    }
    //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else if (threshold / 2 < delta_theta && delta_theta <= 180)
    {
      speed_R = Normal_speed - (delta_theta * Normal_speed / 180);
      speed_L = Normal_speed;
      analogWrite(CH1, speed_R);
      analogWrite(CH3, speed_L);
      Serial.print("turn right");
    }

    //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else
    {
      speed_R = Normal_speed;
      speed_L = Normal_speed - ((360 - delta_theta) * Normal_speed / 180);
      analogWrite(CH1, speed_R);
      analogWrite(CH3, speed_L);
      Serial.print("turn left");
    }
  }
  Serial.print(",");
  Serial.print(speed_L);
  Serial.print(",");
  Serial.println(speed_R);
}

// Hubenyの式

float calculateDistance(double latitude1, double longitude1, double latitude2, double longitude2)
{
  // 先に計算しておいた定数
  double e2 = 0.00669437999019758;   // WGS84における「離心率e」の2乗
  double Rx = 6378137.0;             // WGS84における「赤道半径Rx」
  double m_numer = 6335439.32729246; // WGS84における「子午線曲率半径M」の分子(Rx(1-e^2))

  double rad_lat1 = deg2rad(latitude1);
  double rad_lon1 = deg2rad(longitude1);
  double rad_lat2 = deg2rad(latitude2);
  double rad_lon2 = deg2rad(longitude2);

  float dp = (float) (rad_lon1 - rad_lon2);        // 2点の緯度差
  float dr = (float) (rad_lat1 - rad_lat2);        // 2点の経度差
  float p = (float) ((rad_lon1 + rad_lon2) * 0.5); // 2点の平均緯度

  float w = (float) sqrt(1.0 - e2 * pow(sin(p), 2));
  float m = (float) (m_numer / pow(w, 3)); // 子午線曲率半径
  float n = (float) (Rx / w);              // 卯酉(ぼうゆう)線曲率半径

  // 2点間の距離(単位m)
  float d = (float) sqrt(pow((m * dp), 2) + pow((n * cos(p) * dr), 2));
  return d;
}

// Medianフィルタ関数
int medianFilter()
{
  //ソート用のバッファ
  static int sortBuf[BUF_LEN];

  //ソート用バッファにデータをコピー
  for (int i = 0; i < BUF_LEN; i++)
  {
    sortBuf[i] = buf[i];
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

//=====================================================================================//
void BMX055_Init()
{
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
  Wire.write(0x00); // Normal mode, Sleep duration = 2ms
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
  Wire.write(0x00); // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}
//=====================================================================================//
void BMX055_Gyro()
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
  if (xGyro > 32767)
    xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)
    yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)
    zGyro -= 65536;

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
  if (xMag > 4095)
    xMag -= 8192;
  yMag = ((data[3] << 5) | (data[2] >> 3));
  if (yMag > 4095)
    yMag -= 8192;
  zMag = ((data[5] << 7) | (data[4] >> 1));
  if (zMag > 16383)
    zMag -= 32768;
}
