//This code is derived from https://qiita.com/MergeCells/items/20c3c1a0adfb222a19cd

#include <Wire.h>

//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる

void setup() {
  // put your setup code here, to run once:
  byte DATA_ADDRESS = 0x0000; //書き込むレジスタ(0x0000~0xFFFF全部使える)
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  Serial.begin(9600);
  
  unsigned char p[4];
  int i;
  for (i = 0; i < 4; i++){
    Serial.print(i+1);
    Serial.print("th byte:");
    p[i] = readEEPROM(DEVICE_ADDRESS, DATA_ADDRESS+i);
    Serial.println(p[i]);
  }
  Serial.println("");
  double *d = (double *)p;
  double data = *d;
  Serial.print("ReceiveData:");
  Serial.println(data,6);
}


void loop() {
}

byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}
