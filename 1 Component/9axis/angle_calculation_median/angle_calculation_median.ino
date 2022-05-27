#include<Wire.h>

// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;

int med = 0;
float degree[5] = {0};

//----------------------------------------------------------------------------
//バッファの長さ
#define BUF_LEN 10

//バッファ
int buf[BUF_LEN];
int index = 0;

//フィルター後の値
float filterVal =0;

//----------------------------------------------------------------------------

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);

//----------------------------------------------------------------------------

 //バッファの初期化
  for(int i=0; i<BUF_LEN; i++) {
    buf[i] = 0;
  }
}

  


//----------------------------------------------------------------------------


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

//----------------------------------------------------------------------------



void loop()
{
  
  //BMX055 磁気の読み取り
  BMX055_Mag();
  double Calib = 100; //キャリブレーション用定数
  double Calibx = 10;
  double Caliby = 55;

  float x = atan2(yMag-Caliby,xMag-Calibx)/3.14*180+180;
  x = (x+Calib);

  if (x>360) {
    x = x-360;
  }

  else if (x<0){
   x = x+360;
  }

  else {
   x = x;
 }

 //----------------------------------------------------------------------------

 //バッファに取り込んで、インデックスを更新する。
  buf[index] = x;
  index = (index+1)%BUF_LEN;

//フィルタ後の値を計算

  filterVal = medianFilter();

  //シリアル出力
//  Serial.print("Raw");
//  Serial.print(",");
//  Serial.println("Filtered");
  Serial.print(x);
  Serial.print(",");
  Serial.println(filterVal);

//----------------------------------------------------------------------------
 
//    degree[5] = degree[4];
//    degree[4] = degree[3];
//    degree[3] = degree[2];
//    degree[2] = degree[1];
//    degree[1] = degree[0];
//    degree[0] = x;   //過去5つ分のデータを格納
//    
//    int mx = 0;
//    int mi = 0;
//
//    if(degree[1]>degree[0])mx = 1;
//    else mi = 1;
//    if(degree[2]>degree[mx])mx = 2;   //f(mx)が最大値になる
//    else mi = 2;            //f(mi)が最小値になる
//
//    if(mx==mi)med = degree[1];
//    else med = degree[(3^mx)^mi];//中央値

//    Serial.print("before");
//    Serial.print(",");
//    Serial.println("filtered");
//
//    Serial.print(degree[0]);     //フィルタ前の値
//    Serial.print(",");
//    Serial.print(med);      //フィルタ後の値
//    Serial.print("\n");

 
//Serial.println(x);
//Serial.print(xMag);
//Serial.print(',');
//Serial.print(yMag);
  
  delay(100);
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
