#include<Wire.h>

volatile bool state=false;

//ピンアサイン
const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
const int CH4 = 7;

double Calib = 110; //キャリブレーション用定数
double Calibx = 11.5;
double Caliby = 65;

boolean stop_flag = 0;

// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;

void setup()
{
  // Wire(Arduino-I2C)の初期化
  Wire.begin();
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  //BMX055 初期化
  BMX055_Init();

  pinMode(ENABLE,OUTPUT);  // 7番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 10番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 11版ピンをOUTPUT指定
  pinMode(CH3,OUTPUT);    // 12番ピンをOUTPUT指定
  pinMode(CH4,OUTPUT);    // 13版ピンをOUTPUT指定

  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable
}

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
void BMX055_Mag() {  
  unsigned int data[8];
  
  for (int i = 0; i < 8; i++)
  {     
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb
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



void loop()
{
  //Serial.println("--------------------------------------");
  
  //BMX055 磁気の読み取り
  BMX055_Mag();

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
Serial.println(x);
//Serial.print(xMag);
//Serial.print(',');
//Serial.print(yMag);
     digitalWrite(ENABLE,LOW); // disable
    if(x > 40){ 
      Serial.println("1:Normal rotaion");
      digitalWrite(ENABLE,HIGH); // enable on
      digitalWrite(CH1,HIGH);    
      digitalWrite(CH2,LOW); 
      digitalWrite(CH3,HIGH);    
      digitalWrite(CH4,LOW);
      
    }
    else{

    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    
    }
}
