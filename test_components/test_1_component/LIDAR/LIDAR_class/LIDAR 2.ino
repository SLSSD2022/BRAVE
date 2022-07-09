#include "./LIDAR.h"
#include <HardwareSerial.h>
//=========LIDAR sensor function============================================================================//
LIDAR::LIDAR()//constructer
  :HWSerial(&Serial3)
  ,bytenum(0)
  ,available(1)
  ,distanceUpdated(0)
  ,distancebuf(0)
  ,distance(0)
{
}

LIDAR::LIDAR(HardwareSerial *serialport)//constructer
  :HWSerial(serialport)
  ,bytenum(0)
  ,available(1)
  ,distanceUpdated(0)
  ,distancebuf(0)
  ,distance(0)
{
}

void LIDAR::init(){
  HWSerial->begin(115200);
  while (!HWSerial) {
      // wait for serial port to connect. Needed for native USB port only
  }
}

void LIDAR::encode(byte c)
{
  this->distanceUpdated = 0;
  switch (bytenum) {
    case 0://frame header must be 0x59
      if (c == 0x59) {
        bytenum += 1;
      }
      break;
    case 1://frame header must be 0x59
      if (c == 0x59) {
        bytenum += 1;
      }
      break;
    case 2://distance value low 8 bits
      //        Serial.print("Byte2:");
      //        Serial.println(c,HEX);
      if (c == 0x59) {
        //多分次がcase2
      }
      else {
        this->distancebuf = (unsigned int)c;
        bytenum += 1;
      }
      break;
    case 3://distance value high 8 bits
      //        Serial.print("Byte3:");
      //        Serial.println(c,HEX);
      this->distancebuf = 256 * (unsigned int)c + this->distancebuf;
      bytenum += 1;
      if(this->distancebuf > 0 && this->distancebuf < 1500){
        this->distance = this->distancebuf;
        this->distanceUpdated = 1;
      }
      break;
    case 4://strength value low 8 bits
      bytenum += 1;
      break;
    case 5://strength value high 8 bits
      bytenum += 1;
      break;
    case 6://Temp_L low 8 bits
      bytenum += 1;
      break;
    case 7://Temp_H high 8 bits
      bytenum += 1;
      break;
    case 8://checksum
      bytenum = 0;
      break;
  }
}

unsigned int LIDAR::getDistance(){
  this->distanceUpdated = 0;
  while(this->distanceUpdated == 0){
    while (HWSerial->available() > 0)//near_flagは一時的なもの
    {
      byte c = HWSerial->read();
      this->encode(c);
      if (this->distanceUpdated)
      {
        return this->distance;  // roverの緯度を計算
        break;
      }
    }
  }
}
