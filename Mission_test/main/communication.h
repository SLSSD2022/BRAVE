#ifndef _COMM_HEAD_
#define _COMM_HEAD_

#include "./rover.h"

//------------------------------Communication--------------------------------------------
// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400

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


class TxPacketData{
private:
  uint8_t encodedTx[2 * sizeof(messageStruct)]; //Encoded message to be sent
  const uint8_t generator[4] = {0x46, 0x23, 0x17, 0x0D};
public:
  messageUnion roverPacketData;
  void encodeCyclic();
  void writeToTwelite (IMU*, dataStruct*);

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
};

//Define Structures for receiving goal data & ACK
typedef struct _gpsDataStruct {
  float latA[3];
  float lngA[3];
} gpsDataStruct;

typedef union _gpsPacketUnion {
  gpsDataStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsDataStruct)];
} gpsPacketUnion;

const int MaxBufferSize = 160;

class RxPacketData {
  private:
    const uint8_t parityCheck[3] = {0x5C, 0x72, 0x39};
  public:
    gpsPacketUnion rxData;
    char buffRx[MaxBufferSize];
    uint8_t encodedRx[2 * sizeof(gpsDataStruct)];
    void processData();
    bool checkError(uint8_t dataByte);
    boolean decodeCyclic();
    boolean receiveGPS();
    char encodedReceived[2 * sizeof(messageStruct)];
};

void Parse();
int bufferPos = 0;


#endif
