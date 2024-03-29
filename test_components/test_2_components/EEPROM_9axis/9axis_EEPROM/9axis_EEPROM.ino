//================================================================//
//  AE-BMX055             Arduino UNO                             //
//    VCC                    +5V                                  //
//    GND                    GND                                  //
//    SDA                    A4(SDA)                              //
//    SCL                    A5(SCL)                              //
//                                                                //
//   (JP4,JP5,JP6はショートした状態)                                //
//   http://akizukidenshi.com/catalog/g/gK-13010/                 //
//================================================================//

#include<Wire.h>
// BMX055 加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)
// EEPROM デバイスアドレス(スレーブ)
#define Addr_EEPROM 0x50//1010000

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
  delay(300);
}

void loop()
{
  //BMX055 磁気の読み取り
  BMX055_Mag();

  EEPROM_use();
  
  delay(10);
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
  // Send to PC
  Serial.print("Mag= ");
  Serial.print(xMag);
  Serial.print(",");
  Serial.print(yMag);
  Serial.print(",");
  Serial.print(zMag);
  Serial.println(""); 
}

void EEPROM_use()
{
  // アドレス0番地から5番地までデータを書き込む
  for(int i=0;i<6;i++){
    Wire.beginTransmission(Addr_EEPROM);
    // 対象アドレスに移動
    Wire.write(i >> 8); // Address(High Byte)  
    Wire.write(i & 0xFF); // Address(Low Byte)
    // データの書き込み
    Wire.write(i+1);
    Wire.endTransmission();
    delay(5);
  }
  // アドレス0番地から5番地までデータを読み込む
  for(int i = 0;i<6;i++){
    Wire.beginTransmission(Addr_EEPROM);
    // 対象アドレスに移動
    Wire.write(i >> 8); // Address(High Byte)  
    Wire.write(i & 0xFF); // Address(Low Byte)
    Wire.endTransmission();
     // デバイスへ1byteのレジスタデータを要求する
    Wire.requestFrom(Addr_EEPROM,1);
    while(Wire.available() == 0){
      //null
    }
    Serial.print(Wire.read());
  }
  Serial.println("");
}
