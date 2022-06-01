#include<Wire.h>

// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;
int num = 5000;

//バッファの長さ
#define BUF_LEN 10

//バッファ
int bufx[BUF_LEN];
int bufy[BUF_LEN];
int index = 0;

//フィルター後の値
float filterVal_x =0;
float filterVal_y =0;


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

  return sortBufx[(int)BUF_LEN/2];
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

  return sortBufy[(int)BUF_LEN/2];
}

//クイックソート関数
int quicksortFunc(const void *a, const void *b) {
  return *(int *)a - *(int *)b;
}



void loop()
{
  //適当な大きい数値と小さい数値
  int Min_xMag = 1000;
  int Max_xMag = -1000;
  int Min_yMag = 1000;
  int Max_yMag = -1000;

  Serial.println("Please turn your sensor 360°");
  
  for(int i=1; i < num; i++){ //numで指定した回数数値を取得し、最大値と最小値を更新

     BMX055_Mag();

     bufx[index] = xMag;
     bufy[index] = yMag;

     index = (index+1)%BUF_LEN;

     filterVal_x = medianFilter_x();
     filterVal_y = medianFilter_y();
     
     Min_xMag = min(xMag,Min_xMag);
     Max_xMag = max(xMag,Max_xMag);
     Min_yMag = min(yMag,Min_yMag);
     Max_yMag = max(yMag,Max_yMag);
    
     //delay(100);

  }


  Serial.print("Min xMag =");
  Serial.println(Min_xMag);
  
  Serial.print("Max xMag =");
  Serial.println(Max_xMag);
  
  Serial.print("Min yMag =");
  Serial.println(Min_yMag);
  
  Serial.print("Max yMag =");
  Serial.println(Max_yMag);
  

  int Calib_x = (Max_xMag + Min_xMag) / 2;
  int Calib_y = (Max_yMag + Min_yMag) / 2;

  Serial.print("Calib_x =");
  Serial.println(Calib_x);

  Serial.print("Calib_y =");
  Serial.println(Calib_y);
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
