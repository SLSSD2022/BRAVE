#include <SoftwareSerial.h>
//#include <sercmd.h>

typedef struct dataPacketStruct {       // incoming data packet structure
  uint8_t multiCommsStat;
  uint8_t roverCommsStat;
  uint16_t xMag;
  uint16_t yMag;
  uint16_t calibX;
  uint16_t calibY;
  float x;
  uint16_t cmLong;
  float latR;
  float longR;
  float degRtoA;
  byte statusControl;
  unsigned long int time;
};

typedef struct gpsPacketStruct {
  float latR[3];
  float longR[3];
};

typedef union dataPacketUnion {         // create a union to convert bits to correct data type
  dataPacketStruct message;
  uint8_t packetData[sizeof(dataPacketStruct)];
};

typedef union gpsPacketUnion {
  gpsPacketStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsPacketStruct)];
};

SoftwareSerial MWSerial(2, 3);

// GLOBAL VARIABLE DECLARATIONS
char buffRx[200];
uint8_t encodedRx[100];
uint8_t encodedTx[2 * sizeof(gpsPacketStruct)];
uint8_t lenCtr = 0;
const uint8_t generator[4] = {0x46, 0x23, 0x17, 0x0D};
const uint8_t parityCheck[3] = {0x5C, 0x72, 0x39};
dataPacketUnion dataRx;
gpsPacketUnion gpsTx;

// FUNCTIONS DECLARATION
void processData();
void encodeCyclic();
bool decodeCyclic();
bool checkError(uint8_t dataByte);
void checkRoverStatus();
void printData();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Ground Station Starting\n");

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  digitalWrite(2, LOW);
  delay(100);
  digitalWrite(2, HIGH);

  digitalWrite(3, LOW);
  MWSerial.begin(38400);

  // store GPS coordinates
  gpsTx.gpsData.latR[0] = 14.0;
  gpsTx.gpsData.longR[0] = 121.0;
  gpsTx.gpsData.latR[1] = 14.0;
  gpsTx.gpsData.longR[1] = 121.0;
  gpsTx.gpsData.latR[2] = 14.0;
  gpsTx.gpsData.longR[2] = 121.0;
  encodeCyclic();
}

void loop() {
  // GS waits for valid packet before taking action
  if (MWSerial.available()) {
    lenCtr = 0;
    while (1) {
      char c = MWSerial.read();

      buffRx[lenCtr] = c;
      lenCtr++;
      buffRx[lenCtr] = '\0';

      if (c == '\n' || lenCtr >= 200) {
        break;
      }
    }
  }
  if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '1') {
    if (buffRx[6] == '2') {
      // NACK received, resend GPS
    } else if (buffRx[6] == '0') {
      // incoming data received, process it
      processData();
      if(!decodeCyclic()) {
        checkRoverStatus();
        printData();
      }
    }
  }
}

void processData() {
  // character data is converted to uint8_t data here
  // and is stored in the encodedRx[] buffer
  int i = 7;
  int d, e;
  int checker;

  while (i < lenCtr - 4) {
    checker = buffRx[i] & 0b01000000;
    //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
    if (checker == 0b01000000) {
      d = buffRx[i] + 9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
    }
    else {
      d = buffRx[i];
    }
    d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
    d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
    //Same for the following character
    checker = buffRx[i + 1] & 0b01000000;

    if (checker == 0b01000000) {
      e = buffRx[i + 1] + 9;
    }
    else {
      e = buffRx[i + 1];
    }
    e = e & 0b00001111;
    encodedRx[(i - 7) / 2] = d + e;

    i = i + 2;
  }
}

bool decodeCyclic() {
  uint8_t ctr = 0;
  bool error[2];

  while (ctr < (lenCtr - 11) / 2) {
    encodedRx[2 * ctr] = encodedRx[2 * ctr] & 0x7F;
    encodedRx[2 * ctr + 1] = encodedRx[2 * ctr + 1] & 0x7F;
    error[0] = checkError(encodedRx[2 * ctr]);
    error[1] = checkError(encodedRx[2 * ctr + 1]);
    dataRx.packetData[ctr] = ((encodedRx[2 * ctr] << 1) & 0xF0) +
                             ((encodedRx[2 * ctr + 1] >> 3) & 0x0F);

    if (error[0] || error[1]) {
      // send NACK
      // break loop
      MWSerial.print(":000112X\r\n");
      return true;
    }
    ctr++;
  }

  // no error means this point is reached, send ACK
  MWSerial.print(":000110X\r\n");
  return false;
}

bool checkError(uint8_t dataByte) {
  uint8_t p[3];
  uint8_t ctr = 0;
  p[0] = dataByte & parityCheck[0];
  p[1] = dataByte & parityCheck[1];
  p[2] = dataByte & parityCheck[2];

  while (ctr < 8) {
    p[0] = (p[0] & 1) ^ (p[0] >> 1);
    p[1] = (p[1] & 1) ^ (p[1] >> 1);
    p[2] = (p[2] & 1) ^ (p[2] >> 1);
    ctr++;
  }

  return (p[0] > 0) || (p[1] > 0) || (p[2] > 0);
}

void encodeCyclic() {
  uint8_t ctr = 0;
  uint8_t m;

  while (ctr < sizeof(gpsPacketStruct)) {
    m = gpsTx.gpsBytes[ctr] >> 4;
    encodedTx[2 * ctr] = ((m & 1) * generator[3]) ^ (((m >> 1) & 1) * generator[2]) ^
                             (((m >> 2) & 1) * generator[1]) ^ (((m >> 3) & 1) * generator[0]);
    m = gpsTx.gpsBytes[ctr];
    encodedTx[2 * ctr + 1] = ((m & 1) * generator[3]) ^ (((m >> 1) & 1) * generator[2]) ^
                                 (((m >> 2) & 1) * generator[1]) ^ (((m >> 3) & 1) * generator[0]);
    ctr++;
  }
}

void checkRoverStatus() {
  // check if rover has received GPS data
  // store all data received
  
  if (!(dataRx.message.roverCommsStat&0b01000000)) {
    int ctr = 0;
    
    MWSerial.print(":000110");
    while(ctr<2*sizeof(gpsPacketStruct)){
      if(encodedTx[ctr]<16){
        MWSerial.print("0");
      }
      MWSerial.print(encodedTx[ctr],HEX);
      ctr++;
    }
    MWSerial.print("X\r\n");
  }
}

void printData() {
  Serial.print(dataRx.message.multiCommsStat);
  Serial.print(",");
  Serial.print(dataRx.message.roverCommsStat);
  Serial.print(",");
  Serial.print(dataRx.message.xMag);
  Serial.print(",");
  Serial.print(dataRx.message.yMag);
  Serial.print(",");
  Serial.print(dataRx.message.calibX);
  Serial.print(",");
  Serial.print(dataRx.message.calibY);
  Serial.print(",");
  Serial.print(dataRx.message.x);
  Serial.print(",");
  Serial.print(dataRx.message.cmLong);
  Serial.print(",");
  Serial.print(dataRx.message.latR);
  Serial.print(",");
  Serial.print(dataRx.message.longR);
  Serial.print(",");
  Serial.print(dataRx.message.degRtoA);
  Serial.print(",");
  Serial.print(dataRx.message.statusControl);
  Serial.print(",");
  Serial.print(dataRx.message.time);
}
