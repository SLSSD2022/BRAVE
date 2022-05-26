//This code is derived from https://qiita.com/MergeCells/items/20c3c1a0adfb222a19cd

#include <Wire.h>

//デバイスアドレス(スレーブ)
uint8_t DEVICE_ADDRESS = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
double send_data = -132.434033;

void setup() {
  // put your setup code here, to run once:
  byte DATA_ADDRESS = 0x0000; //書き込むレジスタ(0x0000~0xFFFF全部使える)
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  Serial.begin(9600);
  
  Serial.println("---------Write to EEPROM--------");
  Serial.print("SendData:");
  Serial.println(send_data,6);
  
  EEPROM_write_float(DEVICE_ADDRESS, DATA_ADDRESS, send_data);

  
  Serial.println("---------Read from EEPROM--------");
  double rec_data = EEPROM_read_float(DEVICE_ADDRESS, DATA_ADDRESS);
  Serial.print("ReceiveData:");
  Serial.println(rec_data,6);
}


void loop() {
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

void EEPROM_write_float(int addr_device, unsigned int addr_res, double data){
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
//    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res+i, p[i]);
  }
//  Serial.println("");
}

byte readEEPROM(int addr_device, unsigned int addr_res ) 
{
  byte rdata = 0xFF;
 
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(addr_device,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

double EEPROM_read_float(int addr_device, unsigned int addr_res){
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res+i);
//    Serial.println(p_read[i]);
  }
//  Serial.println("");
  double *d = (double *)p_read;
  double data = *d;
  return data;
}
