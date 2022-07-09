#ifndef _LIDAR_H_
#define _LIDAR_H_
#include <HardwareSerial.h>

class LIDAR{
private:
    HardwareSerial* HWSerial;
    int bytenum;
public:
    boolean available;
    boolean distanceUpdated;
    unsigned int distancebuf;
    unsigned int distance;

    LIDAR();
    LIDAR(HardwareSerial *serialport);
    void init();
    void encode(byte c);
    unsigned int getDistance();
};

#endif
