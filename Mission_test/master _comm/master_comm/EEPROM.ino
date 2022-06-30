#include "./EEPROM.h"
#include <Wire.h>

//============EEPROM function=========================================================================//
void EEPROM::write(int addr_device, unsigned int addr_res, byte data )
{
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);//この遅延はどうやら必要っぽい
}

void EEPROM::writeInt(int addr_device, unsigned int addr_res, int data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    this->write(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EEPROM::writeLong(int addr_device, unsigned int addr_res, unsigned long data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    this->write(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EEPROM::writeFloat(int addr_device, unsigned int addr_res, float data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    this->write(addr_device, addr_res + i, p[i]);
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

void EEPROM::Log() {
  this->writeInt(this->addrEEPROM, this->addrData, imu.xMag);
  this->addrData += 2;
  this->writeInt(this->addrEEPROM, this->addrData, imu.yMag);
  this->addrData += 2;
  this->writeInt(this->addrEEPROM, this->addrData, imu.calibx);
  this->addrData += 2;
  this->writeInt(this->addrEEPROM, this->addrData, imu.caliby);
  this->addrData += 2;
  this->writeFloat(this->addrEEPROM, this->addrData, x);
  this->addrData += 4;
  this->writeInt(this->addrEEPROM, this->addrData, cm_long);
  this->addrData += 2;
  this->writeFloat(this->addrEEPROM, this->addrData, latR);
  this->addrData += 4;
  this->writeFloat(this->addrEEPROM, this->addrData, lngR);
  this->addrData += 4;
  this->writeFloat(this->addrEEPROM, this->addrData, degRtoA);
  this->addrData += 4;
  this->write(this->addrEEPROM, this->addrData, (byte)controlStatus);
  this->addrData += 2;
  overallTime = millis();
  this->writeLong(this->addrEEPROM, this->addrData, overallTime);
  this->addrData += 4;
}

void EEPROM::LogGPSdata() {
  this->writeFloat(this->addrEEPROM, 0, receiveData.rxData.gpsData.latA[0]);
  this->writeFloat(this->addrEEPROM, 4, receiveData.rxData.gpsData.lngA[0]);
  this->writeFloat(this->addrEEPROM, 8, receiveData.rxData.gpsData.latA[1]);
  this->writeFloat(this->addrEEPROM, 12, receiveData.rxData.gpsData.lngA[1]);
  this->writeFloat(this->addrEEPROM, 16, receiveData.rxData.gpsData.latA[2]);
  this->writeFloat(this->addrEEPROM, 20, receiveData.rxData.gpsData.lngA[2]);
}
