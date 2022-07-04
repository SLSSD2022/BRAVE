# ifndef _LIDAR_H_
#define _LIDAR_H_
#include <HardwareSerial.h>

class LIDAR{
private:
    HardwareSerial* HWSerial;
    int bytenum = 0;
public:
    boolean available;
    boolean distanceUpdated = 0;
    unsigned int distancebuf;
    unsigned int distance;

    LIDAR();
    LIDAR(HardwareSerial *serialport);
    unsigned int getDistance();
    void encode(byte c);
}

#endif