#ifndef _GPS_H_
#define _GPS_H_
#include <HardwareSerial.h>
#include <TinyGPS++.h>

class GPS : public TinyGPSPlus{
private:
    HardwareSerial* HWSerial;
public:
    GPS();
    GPS(HardwareSerial *serialport);
    void init();
    void updateGPSlocation(float* lat,float* lng);
}
#endif 