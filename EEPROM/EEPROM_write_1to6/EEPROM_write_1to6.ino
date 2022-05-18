#include <Wire.h>
//デバイスアドレス(スレーブ)
uint8_t ADDRESS = 0x50;//1010000
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // マスタとしてI2Cバスに接続する
  Wire.begin();
  // アドレス0番地から5番地までデータを書き込む
  for(int i=0;i<6;i++){
    Wire.beginTransmission(ADDRESS);
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
    Wire.beginTransmission(ADDRESS);
    // 対象アドレスに移動
    Wire.write(i >> 8); // Address(High Byte)  
    Wire.write(i & 0xFF); // Address(Low Byte)
    Wire.endTransmission();
     // デバイスへ1byteのレジスタデータを要求する
    Wire.requestFrom(ADDRESS,1);
    while(Wire.available() == 0){
      //null
    }
    Serial.println(Wire.read());
  }
}
void loop() {
  // put your main code here, to run repeatedly:
}
