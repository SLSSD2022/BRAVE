#include <Tinygps++.h>
#include <HardwareSerial.h>
#include "./GPS.h"

//=========GPS and position function============================================================================//
GPS::GPS()
    :HWSerial(&Serial1)
{
}

GPS::GPS(HardwareSerial *serialport)
    :HWSerial(serialport)
{
}

void GPS::init(){
    HWSerial->begin(115200);
    while (!HWSerial) {
        // wait for serial port to connect. Needed for native USB port only
    }
}

void GPS::updateGPSlocation(float* lat,float* lng) {
  while (HWSerial->available() > 0)
  {
    //    Serial.print("YES");
    char c = HWSerial->read();
    //    Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated())
    {
      Serial.println("");
      Serial.println("I got new GPS!");
      *lat = gps.location.lat();  // roverの緯度を計算
      *lng = gps.location.lng(); // roverの経度を計算
      break;
    }
    //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
    delay(1);
  }
}

