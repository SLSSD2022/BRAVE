#ifndef _COMM_HEAD_
#define _COMM_HEAD_

#include "./rover.h"

//------------------------------Communication--------------------------------------------
// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400

//Define Structures for receiving and Handling Rover data

class TxPacketData : public RoverData{
  private:
    uint8_t encodedTx[2 * sizeof(roverDataPacket)]; //Encoded message to be sent
    const uint8_t generator[4] = {0x46, 0x23, 0x17, 0x0D};
  public:
    void encodeCyclic();
    void  writeToTwelite (bmx055 imu, float x, uint16_t distance, float latR, float lngR, float degRtoA, byte controlStatus, unsigned long int overallTime);
};

const int MaxBufferSize = 160;
typedef struct _rxPacketData {
  gpsPacketUnion rxData;
  const uint8_t parityCheck[3] = {0x5C, 0x72, 0x39};
  char buffRx[MaxBufferSize];
  uint8_t encodedRx[2 * sizeof(gpsDataStruct)];
  void processData();
  bool checkError(uint8_t dataByte);
  boolean decodeCyclic();
  boolean receiveGPS();
  char encodedReceived[2 * sizeof(roverDataPacket)];
} rxPacketData;

void Parse();
int bufferPos = 0;

TxPacketData sendData; //Packet to be coded and then written to twelite
// packetData packetTx;//Packet to be coded and then written to twelite
rxPacketData receiveData; //Received packet with GPS data
// gpsPacketUnion dataRx;//Received packet with GPS data


unsigned long start;
unsigned long stopi;

#endif
