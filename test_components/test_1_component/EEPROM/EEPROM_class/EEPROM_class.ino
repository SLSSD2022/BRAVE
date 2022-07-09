//This code is derived from https://qiita.com/MergeCells/items/20c3c1a0adfb222a19cd
#include <Wire.h>
#include "./EEPROM.h"
EEPROM eeprom(0x50);//24lC1025の場合1010000(前半)or1010100(後半)を選べる
float send_data = -132.434033;
uint8_t DATA_ADDRESS = 0x0000; //書き込むレジスタ(0x0000~0xFFFF全部使える)

void setup() {
  Serial.begin(9600);



  eeprom.init(0x0000);
  
  Serial.println("---------Write to EEPROM--------");
  Serial.print("SendData:");
  Serial.println(send_data,6);
  eeprom.writeFloat(DATA_ADDRESS, send_data);
  
  Serial.println("---------Read from EEPROM--------");
  float rec_data = eeprom.readFloat(DATA_ADDRESS);
  Serial.print("ReceiveData:");
  Serial.println(rec_data,6);
}


void loop() {
}
