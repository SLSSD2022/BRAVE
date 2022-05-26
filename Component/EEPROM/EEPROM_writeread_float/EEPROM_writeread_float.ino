//This code is derived from https://qiita.com/MergeCells/items/20c3c1a0adfb222a19cd

#include <Wire.h>

//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
double d = -122.497033;

void setup() {
  // put your setup code here, to run once:
  byte DATA_ADDRESS = 0x0000; //書き込むレジスタ(0x0000~0xFFFF全部使える)
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  Serial.begin(9600);

  Serial.print("SendData:");
  Serial.println(d,6);
  Serial.println("---------Write to EEPROM--------");
  
  unsigned char *p = (unsigned char *)&d;
  int i;
  for (i = 0; i < (int)sizeof(d); i++){
    Serial.print(i+1);
    Serial.print("th byte:");
    Serial.println(p[i]);
    writeEEPROM(DEVICE_ADDRESS, DATA_ADDRESS+i, p[i]);
  }
  Serial.println("");

  
  Serial.println("---------Read from EEPROM--------");
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++){
    Serial.print(i+1);
    Serial.print("th byte:");
    p_read[i] = readEEPROM(DEVICE_ADDRESS, DATA_ADDRESS+i);
    Serial.println(p_read[i]);
  }
  Serial.println("");
  double *d = (double *)p_read;
  double data = *d;
  Serial.print("ReceiveData:");
  Serial.println(data,6);
}


void loop() {
  // put your main code here, to run repeatedly:
//  int n = (int)sizeof(d);
//  unsigned char *p = (unsigned char *)&d;
//  int i;
//  Serial.print("02X ");
//  for (i = 0; i < (int)sizeof(d); i++){
//    Serial.print( p[i],HEX);
//    Serial.print(":");
//  }
//  Serial.println(d);
//  d = -(d*1.1 + 0.04);
//  delay(1000);
}

void writeEEPROM(int addr_device, unsigned int addr_res, byte data ) 
{
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
 
  delay(5);
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
