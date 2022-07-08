#ifndef _COMM_HEAD_
#define _COMM_HEAD_

#include <HardwareSerial.h>
//------------------------------Communication--------------------------------------------

//Define Structures for Handling Rover data
typedef struct _messageStruct {
  uint8_t roverComsStat;
  int xMag;
  int yMag;
  uint16_t calibx;
  uint16_t caliby;
  float x;
  uint16_t cmBottom;
  uint16_t cmHead;
  uint16_t cmLong;
  uint16_t cmLidar;
  float latA;
  float lngA;
  float latR;
  float lngR;
  float degRtoA;
  float rangeRtoA;
  byte motorControl;
  unsigned long int overallTime;
} messageStruct;


typedef union _messageUnion {
  messageStruct message;
  unsigned char packetData[sizeof(messageStruct)];
} messageUnion;

//Define Structures for receiving goal data & ACK
typedef struct _gpsDataStruct {
  float latA[3];
  float lngA[3];
} gpsDataStruct;

typedef union _gpsPacketUnion {
  gpsDataStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsDataStruct)];
} gpsPacketUnion;


#define MaxBufferSize 160

class Communication
{
private:
    //setup
    HardwareSerial* HWSerial;
    uint8_t BPS;
    uint8_t RST;

    //encode
    uint8_t encodedTx[2 * sizeof(messageStruct)]; //Encoded message to be sent
    uint8_t statusTx[2]; //Encoded Communication status to be sent (only) in inital stages
    const uint8_t generator[4] = {0x46, 0x23, 0x17, 0x0D};

    //decode
    char buff[MaxBufferSize];
    int bufferPos = 0;
    uint8_t encodedRx[2 * sizeof(gpsDataStruct)];
    const uint8_t parityCheck[3] = {0x5C, 0x72, 0x39};

public:
    //setup    
    Communication();
    Communication(HardwareSerial *serialPort);
    Communication(HardwareSerial *serialPort,uint8_t BPS,uint8_t RST);
    void init();

    //transmission
    messageUnion roverPacketData;
    void encodeCyclic();
    void encodeCyclicStatus();
    void writeToTwelite (IMU*, dataStruct*);
    void HKtoGS(IMU*, dataStruct*);
    void sendStatus();
    
    //update
    void initializeRoverComsStat();
    void updateRoverComsStat(byte statusUpdate);
    void updateGoalStat();

    void setAllData(IMU*, dataStruct*);
    void setMag(IMU*);
    void setCalib(IMU*);
    void setAttitude(float x);
    void setDistByLIDAR(uint16_t cm_LIDAR);
    void setPosition(float latR, float lngR);
    void setDegRtoA(float degRtoA);
    void setControlStatus(byte controlStatus);
    void setTime(unsigned long int overallTime);

    void printAllData();
    void printRoverComsStat();
    void printMag();
    void printCalib();
    void printAttitude();
    void printDistByLIDAR();
    void printPosition();
    void printDegRtoA();
    void printControlStatus();
    void printTime();

    //receive
    gpsPacketUnion gpsPacket;
    char encodedReceived[2 * sizeof(messageStruct)];
    void processData();
    bool checkError(uint8_t dataByte);
    boolean decodeCyclic();
    boolean receiveGPS();
};

void Parse();


#endif
