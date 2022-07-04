#include "./EEPROM.h"
#include <Wire.h>
#include "./rover.h"
#include "./IMU.h"
#include "./communication.h"

//============EEPROM function=========================================================================//

void EEPROM::init()
{
  this->addrEEPROM = 0x50; //24lC1025の場合1010000(前半)or1010100(後半)を選べる
  this->addrData = 30; //書き込むレジスタ(0x0000~0xFFFF全部使える) (0~30は目的地のGPSデータとステータスを保管する)
}

void EEPROM::setAddr(uint8_t addr)
{
  this->addrEEPROM = addr;
}

void EEPROM::write(unsigned int addr_res, byte data )
{
  Wire.beginTransmission(this->addrEEPROM);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);//この遅延はどうやら必要っぽい
}

void EEPROM::writeInt(unsigned int addr_res, int data) {
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


void EEPROM::writeLong(unsigned int addr_res, unsigned long data) {
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


void EEPROM::writeFloat(unsigned int addr_res, float data) {
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

byte EEPROM::read(int addr_device, unsigned int addr_res )
{
  byte rdata = 0xFF;

  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(addr_device, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


float EEPROM::readFloat(int addr_device, unsigned int addr_res) {
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    p_read[i] = this->read(addr_device, addr_res + i);
    //    Serial.println(p_read[i]);
  }
  //  Serial.println("");
  float *d = (float *)p_read;
  float data = *d;
  return data;
}

void EEPROM::log() {
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

void EEPROM::logGPSdata() {
  this->writeFloat(0, comm.gpsPacket.gpsData.latA[0]);
  this->writeFloat(4, comm.gpsPacket.gpsData.lngA[0]);
  this->writeFloat(8, comm.gpsPacket.gpsData.latA[1]);
  this->writeFloat(12, comm.gpsPacket.gpsData.lngA[1]);
  this->writeFloat(16, comm.gpsPacket.gpsData.latA[2]);
  this->writeFloat(20, comm.gpsPacket.gpsData.lngA[2]);
}
