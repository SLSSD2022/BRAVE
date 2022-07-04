# ifndef _LIDAR_H_
#define _LIDAR_H_

class LIDAR{
private:
    int bytenum = 0;
public:
    boolean available;
    boolean distanceUpdated = 0;
    unsigned int distancebuf;
    unsigned int distance;
    LIDAR();
    void encode(byte c);
}
#endif