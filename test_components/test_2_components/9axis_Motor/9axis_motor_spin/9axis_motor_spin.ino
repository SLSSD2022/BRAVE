#include<Wire.h>
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)


// センサーの値を保存するグローバル変数
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;


//モーター
const int ENABLE = 8;
const int CH1 = 11;
const int CH2 = 9;
const int CH3 = 12;
const int CH4 = 10;

double Calib; //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calibx;
double Caliby;

int Speed = 100;


//メディアンフィルタ用バッファの長さ
#define BUF_LEN 10

//メディアンフィルタ用バッファ
int buf[BUF_LEN];
int index = 0;

//メディアンフィルター後の値
float filterVal =0;

//磁気キャリブ用のバッファの長さ
#define MAG_BUF_LEN 400

//磁気キャリブ用のバッファ
int xMag_buf[MAG_BUF_LEN];
int yMag_buf[MAG_BUF_LEN];
int Mag_index = 0;


float deg2rad(float deg) {
  return deg * PI / 180.0;
}


void setup()
{
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  //BMX055 初期化
  BMX055_Init();
  //バッファの初期化
  for(int i=0; i<BUF_LEN; i++) {
    buf[i] = 0;
  }
  
  pinMode(ENABLE,OUTPUT);
  pinMode( CH1, OUTPUT );
  pinMode( CH2, OUTPUT );
  pinMode( CH3, OUTPUT );
  pinMode( CH4, OUTPUT );
  digitalWrite(ENABLE,LOW); // disable
  
  delay(1000);
  digitalWrite(ENABLE,HIGH); // enable
  digitalWrite( CH2, LOW);
  digitalWrite( CH4, LOW);
}
  

void loop()
{
  //常にスピンさせておく。
  digitalWrite(ENABLE,HIGH); // enable on
  analogWrite(CH1, Speed);    
  digitalWrite(CH2,LOW); 
  digitalWrite(CH3,LOW);    
  analogWrite(CH4, Speed); 
  
  //BMX055 磁気の読み取り
  BMX055_Mag();

  // バッファに取り込んで、インデックスを更新する。
  xMag_buf[Mag_index] = xMag;
  yMag_buf[Mag_index] = yMag;
  Mag_index = (Mag_index+1)%MAG_BUF_LEN;

  //キャリブレーション値を計算する
  Calibx = center_calculation(xMag_buf,MAG_BUF_LEN);
  Caliby = center_calculation(yMag_buf,MAG_BUF_LEN);
  
  Serial.print(Calibx);
  Serial.print(",");
  Serial.print(Caliby);
  Serial.print(",");

  Serial.print(xMag-Calibx);
  Serial.print(",");
  Serial.print(yMag-Caliby);
  Serial.print(",");

  double x = angle_calculation();
  
  Serial.print(x);
  Serial.print(",");

  
 // バッファに取り込んで、インデックスを更新する。
  buf[index] = x;
  index = (index+1)%BUF_LEN;

//フィルタ後の値を計算

  filterVal = medianFilter();
  return filterVal;

  
  Serial.print("filterVal:");
  Serial.print(filterVal);
  Serial.print(":");
}


double angle_calculation(){
  double x = atan2(yMag-Caliby,xMag-Calibx)/3.14*180+180; 
  x = x + Calib;

  x = x - 7; 

  if (x>360) {
    x = x-360;
  }

  else if (x<0){
    x = x +360;
  }

  else {
    x = x;
  }

  return x;
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

//最大値と最小値の平均を出す関数
int center_calculation(int* buf,int len) {
  //ソート用のバッファ
  int sortBuf[len];

  //ソート用バッファにデータをコピー
  for(int i=0; i<len; i++) {
    sortBuf[i] = buf[i];
  }

  //クイックソートで並べ替える
  qsort(sortBuf, len, sizeof(int), quicksortFunc);

  return (sortBuf[0] + sortBuf[(int)BUF_LEN-1])/2;
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
