#include <SoftwareSerial.h>
//#include <sercmd.h>

const int RST = 4;


typedef struct dataPacketStruct {       // incoming data packet structure
  uint8_t multiCommsStat;
  uint8_t roverCommsStat;
  int16_t xMag;
  int16_t yMag;
  float calibX;
  float calibY;
  float x;
  uint8_t cmBottom;
  uint8_t cmHead;
  uint16_t cmLong;
  uint16_t cmLidar;
  float latA;
  float lngA;
  float latR;
  float lngR;
  float degRtoA;
  float rangeRtoA;
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
char buffRx[250];
uint8_t encodedRx[2*sizeof(dataPacketStruct)];
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

  pinMode(RST,OUTPUT);

  digitalWrite(RST, LOW);
  delay(100);
  digitalWrite(RST, HIGH);
  
  MWSerial.begin(38400);

  // store GPS coordinates
  gpsTx.gpsData.latR[0] = 35.720033;
  gpsTx.gpsData.longR[0] = 139.736089;
  gpsTx.gpsData.latR[1] = 35.7199481;
  gpsTx.gpsData.longR[1] = 139.7359838;
  gpsTx.gpsData.latR[2] = 35.7199134;
  gpsTx.gpsData.longR[2] = 139.7362514;
  encodeCyclic();
}
boolean gps_flag = 1;
int initialTime = 0;
void loop() {
  // GS waits for valid packet before taking action

  if (MWSerial.available() > 0) {
    char c = MWSerial.read();
    if ( c != '\n' && (lenCtr < 200 - 1) ){
      buffRx[lenCtr] = c;
      lenCtr++; 
        }
  else
  {
    buffRx[lenCtr] = '\0';
    if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '1') {
      //Serial.println("------GS Packet ------");
      //Serial.println(buffRx);
      if (buffRx[6] == '2') {
        // NACK received, resend GPS
        Serial.println("------NACK ------");
        /* MWSerial.print(":000110");
        int ctr1=0;
        while (ctr1<2*sizeof(gpsPacketStruct)){
          if((uint8_t)encodedTx[ctr1]<16){
            MWSerial.print("0");
          }
          MWSerial.print(encodedTx[ctr1],HEX);
          ctr1++;
        }
        MWSerial.print("X\r\n");*/
      }
      else if (buffRx[6] == '0') {
        Serial.println("------Rover Data ------");
        // incoming data received, process it
        processData();
        if(!decodeCyclic()) {
          checkRoverStatus();
          printData();
        }
      }
      lenCtr = 0;
    }
    lenCtr = 0;
  }
}

}

void processData() {
  // character data is converted to uint8_t data here
  // and is stored in the encodedRx[] buffer
  int i = 7;
  int d, e;
  int checker;
  Serial.print("lenCtr: " );
  Serial.println(lenCtr);

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

  while (ctr < sizeof(dataPacketStruct)) {
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
  MWSerial.print(":000111X\r\n");
  /*int ctr2 =0;
  while (ctr2< sizeof(dataPacketStruct)){
    Serial.println(dataRx.packetData[ctr2],HEX);
    ctr2++;
  }*/
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
  byte stat = dataRx.message.roverCommsStat;
  if (stat == 0b11111101){
    Serial.println("Mission Success!");
    Serial.println("Rover Mission End");
    return;
  }  else if (stat == 0b11111100) {
    Serial.println("Final Goal Reached!");
    return;
    }  else if (stat == 0b11111010) {
    Serial.println("Moving to Final Goal");
    return;
    }  else if (stat == 0b11111000) {
    Serial.println("Second Goal Reached!");
    return;
    }  else if (stat == 0b11110110) {
    Serial.println("Moving to Second Goal!");
    return;
    }  else if (stat == 0b11110100) {
    Serial.println("First Goal Reached!");
    return;
    }  else if (stat == 0b11110010) {
    Serial.println("Calculation Finished");
    Serial.println("Moving to First Goal!");
    return;
    }      else if (stat == 0b11110000) {
    Serial.println("GPS Coordinates Successfully received by Rover");
    return;
  } else if (stat==0b11100000) {
    int ctr = 0;
    Serial.println("Distancing detected");
    Serial.println("Sending GPS coordinates...");
    MWSerial.print(":000110");
    while(ctr<2*sizeof(gpsPacketStruct)){
      if(encodedTx[ctr]<16){
        MWSerial.print("0");
      }
      MWSerial.print(encodedTx[ctr],HEX);
      ctr++;
    }
    MWSerial.print("X\r\n");
    return;
  } else if (stat == 0b11000000) {
    Serial.println("Separation between Multicopter and Rover Detected");
    Serial.println("Moving to distancing stage...");
    return;
  } else if (stat == 0b10000000){
    Serial.println("Landing Detected");
    Serial.println("Moving to Separation stage...");
    return;
  }
}


void printData() {
  Serial.print("Multicopter Comms Status: ");
  Serial.println(dataRx.message.multiCommsStat,HEX);
  Serial.print("rover Comms Status: ");
  Serial.println(dataRx.message.roverCommsStat,HEX);
  Serial.print("x Mag: ");
  Serial.println(dataRx.message.xMag);
  Serial.print("y Mag: ");
  Serial.println(dataRx.message.yMag);
  Serial.print("Calibration x: ");
  Serial.println(dataRx.message.calibX);
  Serial.print("Calibration y: ");
  Serial.println(dataRx.message.calibY);
  Serial.print("Body-axis Attitude: ");
  Serial.println(dataRx.message.x);
  Serial.print("Range bottom: ");
  Serial.println(dataRx.message.cmBottom);
  Serial.print("Range Head: ");
  Serial.println(dataRx.message.cmHead);    
  Serial.print("Range Sensor reading: ");
  Serial.println(dataRx.message.cmLong);
  Serial.print("Range Lidar: ");
  Serial.println(dataRx.message.cmLidar);  
  Serial.print("Next Goal's Lattitude: ");
  Serial.println(dataRx.message.latA);
  Serial.print("Next Goal's Longitude: ");
  Serial.println(dataRx.message.lngA);  
  Serial.print("Current Lattitude: ");
  Serial.println(dataRx.message.latR);
  Serial.print("Current Longitude: ");
  Serial.println(dataRx.message.lngR);
  Serial.print("Absolute angle to Destination: ");
  Serial.println(dataRx.message.degRtoA);
  Serial.print("Absolute distance to Destination: ");
  Serial.println(dataRx.message.rangeRtoA);  
  Serial.print("Motor Status Control: ");
  Serial.println(dataRx.message.statusControl);
  Serial.print("Time: ");
  Serial.println(dataRx.message.time);
}

 /* OLD CODE
 
  int ctr1=0;
  if (gps_flag==1){
    MWSerial.print(":000110");
    //Serial.print(":000110");
  while (ctr1<2*sizeof(gpsPacketStruct)){
    if((uint8_t)encodedTx[ctr1]<16){
      MWSerial.print("0");
      //Serial.print("0");
    }
    MWSerial.print(encodedTx[ctr1],HEX);
    //Serial.print(encodedTx[ctr1],HEX);
    ctr1++;
  }
  MWSerial.print("X\r\n");
  //Serial.print("X\r\n");

  initialTime ++;
  }
  if (initialTime > 50){
    gps_flag = 0;
  }*/
