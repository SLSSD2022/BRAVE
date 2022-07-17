#include "./EXPROM.h"
#include <Wire.h>
//#include "./rover.h"
//#include "./IMU.h"
//#include "./communication.h"

//============EXPROM function=========================================================================//

EXPROM::EXPROM()
  :addrEXPROM(0x50)//24lC1025の場合1010000(前半)or1010100(後半)を選べる
{
}

EXPROM::EXPROM(byte addr)
  :addrEXPROM(addr)
{
}

void EXPROM::init(byte addr)
{
  //マスタとしてI2Cバスに接続する
  Wire.begin();
  this->addrData = addr; //書き込むレジスタ(0x0000~0xFFFF全部使える) (0~30は目的地のGPSデータとステータスを保管する)
}


void EXPROM::write(uint8_t addr_res, byte data )
{
  Wire.beginTransmission(this->addrEXPROM);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);//この遅延はどうやら必要っぽい
}

void EXPROM::writeInt(uint8_t addr_res, int data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    this->write(addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EXPROM::writeLong(uint8_t addr_res, unsigned long data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    this->write(addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EXPROM::writeFloat(uint8_t addr_res, float data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    this->write(addr_res + i, p[i]);
  }
  //  Serial.println("");
}

byte EXPROM::read(uint8_t addr_res )
{
  byte rdata = 0xFF;

  Wire.beginTransmission(this->addrEXPROM);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(this->addrEXPROM,(uint8_t) 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


float EXPROM::readFloat(uint8_t addr_res) {
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    p_read[i] = this->read(addr_res + i);
    //    Serial.println(p_read[i]);
  }
  //  Serial.println("");
  float *d = (float *)p_read;
  float data = *d;
  return data;
}

void EXPROM::log() {
  this->writeInt(this->addrData, imu.xMag);
  this->addrData += 2;
  this->writeInt(this->addrData, imu.yMag);
  this->addrData += 2;
  this->writeInt(this->addrData, imu.calibx);
  this->addrData += 2;
  this->writeInt(this->addrData, imu.caliby);
  this->addrData += 2;
  this->writeFloat(this->addrData, rover.data.x);
  this->addrData += 4;
  this->writeInt(this->addrData, rover.data.cmLong);
  this->addrData += 2;
  this->writeFloat(this->addrData, rover.data.latR);
  this->addrData += 4;
  this->writeFloat(this->addrData, rover.data.lngR);
  this->addrData += 4;
  this->writeFloat(this->addrData, rover.data.degRtoA);
  this->addrData += 4;
  this->write(this->addrData, (byte)rover.data.motorControl);
  this->addrData += 2;
  this->writeLong(this->addrData, rover.data.overallTime);
  this->addrData += 4;
}

void EXPROM::logGPSdata() {
  this->writeFloat(0, comm.gpsPacket.gpsData.latA[0]);
  this->writeFloat(4, comm.gpsPacket.gpsData.lngA[0]);
  this->writeFloat(8, comm.gpsPacket.gpsData.latA[1]);
  this->writeFloat(12, comm.gpsPacket.gpsData.lngA[1]);
  this->writeFloat(16, comm.gpsPacket.gpsData.latA[2]);
  this->writeFloat(20, comm.gpsPacket.gpsData.lngA[2]);
}
