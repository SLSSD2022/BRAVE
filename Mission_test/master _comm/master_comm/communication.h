#ifndef _COMM_HEAD_
#define _COMM_HEAD_

//------------------------------Communication--------------------------------------------
// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400

//Define Structures for receiving and Handling Rover data

typedef union _packetData {
  roverData message;
  unsigned char packetData[sizeof(roverData)];
} packetData;

const int MaxBufferSize = 160;
char buffRx[MaxBufferSize];
void Parse();
int bufferPos = 0;
packetData packetTx;//Packet to be coded and then written to twelite
gpsPacketUnion dataRx;//Received packet with GPS data

void  writeToTwelite();
void encodeCyclic();
uint8_t encodedTx[2 * sizeof(roverData)]; //Encoded message to be sent
uint8_t encodedRx[2 * sizeof(gpsDataStruct)];
const uint8_t generator[4] = {0x46, 0x23, 0x17, 0x0D};
const uint8_t parityCheck[3] = {0x5C, 0x72, 0x39};
unsigned long start;
unsigned long stopi;
char encodedReceived[2 * sizeof(roverData)];

void processData();
void encodeCyclic();
bool checkError(uint8_t dataByte);
boolean decodeCyclic();
void  writeToTwelite ();
boolean receiveGPS();

#endif