//This code is derived from https://qiita.com/MergeCells/items/20c3c1a0adfb222a19cd

#include <Wire.h>

//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる

void setup() {
  // put your setup code here, to run once:
  byte DATA_ADDRESS = 0x00FF; //読み込むレジスタ(0x0000~0xFFFF全部使える)
  Serial.begin(9600);
  
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(DATA_ADDRESS >> 8); // Address(High Byte)
  Wire.write(DATA_ADDRESS & 0xFF);  // Address(Low Byte)
  Wire.endTransmission();
  
  // デバイスへ1byteのレジスタデータを要求する
  Wire.requestFrom(DEVICE_ADDRESS,1);
  Serial.println(Wire.read());
}

void loop() {
  // put your main code here, to run repeatedly:
}
