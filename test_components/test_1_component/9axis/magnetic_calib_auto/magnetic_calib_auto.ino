#include<Wire.h>

// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;

//バッファの長さ
#define BUF_LEN 400

//バッファ
int bufx[BUF_LEN];
int bufy[BUF_LEN];
int index = 0;

double Calib = 175; //キャリブレーション用定数(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calibx = 20;
double Caliby = 133;


void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();
  delay(300);

   //バッファの初期化
  for(int i=0; i<BUF_LEN; i++) {
    bufx[i] = 0;
    bufy[i] = 0;
  }
}




void loop()
{
  //適当な大きい数値と小さい数値
  int Min_xMag = 1000;
  int Max_xMag = -1000;
  int Min_yMag = 1000;
  int Max_yMag = -1000;

//  Serial.println("Please turn your sensor 360°");
  
  BMX055_Mag();
  bufx[index] = xMag-Calibx;
  bufy[index] = yMag-Caliby;
  index = (index+1)%BUF_LEN;
  
  Serial.print("xMag:");
  Serial.print(xMag-Calibx);
  int Calib_x = medianFilter_x();

  Serial.print(":Calib_x:");
  Serial.print(Calib_x);
  
  Serial.print(":yMag:");
  Serial.print(yMag-Caliby);
  int Calib_y = medianFilter_y(); 

  Serial.print(":Calib_y:");
  Serial.println(Calib_y);
}
  

//Medianフィルタ関数
int medianFilter_x() {
  //ソート用のバッファ
  static int sortBufx[BUF_LEN];

  //ソート用バッファにデータをコピー
  for(int i=0; i<BUF_LEN; i++) {
    sortBufx[i] = bufx[i];
  }

  //クイックソートで並べ替える
  qsort(sortBufx, BUF_LEN, sizeof(int), quicksortFunc);
  
  Serial.print(":Min:");
  Serial.print(sortBufx[0]);
  Serial.print(":Max:");
  Serial.print(sortBufx[BUF_LEN-1]);

  return (sortBufx[0]+sortBufx[BUF_LEN-1])/2;
}

//Medianフィルタ関数
int medianFilter_y() {
  //ソート用のバッファ
  static int sortBufy[BUF_LEN];

  //ソート用バッファにデータをコピー
  for(int i=0; i<BUF_LEN; i++) {
    sortBufy[i] = bufy[i];
  }

  //クイックソートで並べ替える
  qsort(sortBufy, BUF_LEN, sizeof(int), quicksortFunc);

  Serial.print(":Min:");
  Serial.print(sortBufy[0]);
  Serial.print(":Max:");
  Serial.print(sortBufy[BUF_LEN-1]);

  return (sortBufy[0]+sortBufy[BUF_LEN-1])/2;
}

//クイックソート関数
int quicksortFunc(const void *a, const void *b) {
  return *(int *)a - *(int *)b;
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
