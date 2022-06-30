#include "./EEPROM.h"
#include <Wire.h>

//============EEPROM function=========================================================================//
void writeEEPROM(int addr_device, unsigned int addr_res, byte data )
{
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);//この遅延はどうやら必要っぽい
}

void EEPROM_write_int(int addr_device, unsigned int addr_res, int data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EEPROM_write_long(int addr_device, unsigned int addr_res, unsigned long data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EEPROM_write_float(int addr_device, unsigned int addr_res, float data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res + i, p[i]);
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

  Wire.requestFrom(addr_device, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


float EEPROM_read_float(int addr_device, unsigned int addr_res) {
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res + i);
    //    Serial.println(p_read[i]);
  }
  //  Serial.println("");
  float *d = (float *)p_read;
  float data = *d;
  return data;
}

void LogToEEPROM() {
  EEPROM_write_int(addrEEPROM, addrData, IMU.xMag);
  addrData += 2;
  EEPROM_write_int(addrEEPROM, addrData, IMU.yMag);
  addrData += 2;
  EEPROM_write_int(addrEEPROM, addrData, IMU.calibx);
  addrData += 2;
  EEPROM_write_int(addrEEPROM, addrData, IMU.caliby);
  addrData += 2;
  EEPROM_write_float(addrEEPROM, addrData, x);
  addrData += 4;
  EEPROM_write_int(addrEEPROM, addrData, cm_long);
  addrData += 2;
  EEPROM_write_float(addrEEPROM, addrData, latR);
  addrData += 4;
  EEPROM_write_float(addrEEPROM, addrData, lngR);
  addrData += 4;
  EEPROM_write_float(addrEEPROM, addrData, degRtoA);
  addrData += 4;
  writeEEPROM(addrEEPROM, addrData, (byte)controlStatus);
  addrData += 2;
  overallTime = millis();
  EEPROM_write_long(addrEEPROM, addrData, overallTime);
  addrData += 4;
}

void LogGPSdata() {
  EEPROM_write_float(addrEEPROM, 0, receiveData.rxData.gpsData.latA[0]);
  EEPROM_write_float(addrEEPROM, 4, receiveData.rxData.gpsData.lngA[0]);
  EEPROM_write_float(addrEEPROM, 8, receiveData.rxData.gpsData.latA[1]);
  EEPROM_write_float(addrEEPROM, 12, receiveData.rxData.gpsData.lngA[1]);
  EEPROM_write_float(addrEEPROM, 16, receiveData.rxData.gpsData.latA[2]);
  EEPROM_write_float(addrEEPROM, 20, receiveData.rxData.gpsData.lngA[2]);
}
