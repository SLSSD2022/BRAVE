//This code is derived from https://qiita.com/MergeCells/items/20c3c1a0adfb222a19cd

#include <Wire.h>

//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる

void setup() {
  // put your setup code here, to run once:
  byte DATA_ADDRESS = 0x00FF; //書き込むレジスタ(0x0000~0xFFFF全部使える)
  byte i = 0xAA; //書き込む内容(1バイト)
  Serial.begin(9600);
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(DATA_ADDRESS >> 8);  // Address(High Byte)
  Wire.write(DATA_ADDRESS & 0xFF); // Address(Low Byte)
  // データの書き込み
  Wire.write(i);//送信バッファにデータを貯める(配列とデータ長を引数として複数byte送信可能)
  Wire.endTransmission();//データを送信する．実はこの関数を呼ぶまでデータは送信していない．
}


void loop() {
  // put your main code here, to run repeatedly:
  
}
