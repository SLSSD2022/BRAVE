#include <SoftwareSerial.h>
//#include <sercmd.h>

const int RST = 4;

typedef union rcPacketUnion {
  float rc[16];
  uint8_t rcBytes[16*sizeof(float)];
};

SoftwareSerial MWSerial(2, 3);

// GLOBAL VARIABLE DECLARATIONS
char buffRx[250];
uint8_t lenCtr = 0;
rcPacketUnion rcPacket;

// FUNCTIONS DECLARATION
void processData();
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

}

void loop() {
  // GS waits for valid packet before taking action
  //long int tstart = millis();
  if (MWSerial.available() > 0) {
    char c = MWSerial.read();
    if ( c != '\n' && (lenCtr < 250 - 1) ){
      buffRx[lenCtr] = c;
      //Serial.print(c);
      lenCtr++; 
        }
  else
  {
    //Serial.println();
    buffRx[lenCtr] = '\0';
    if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '1' && buffRx[6] == '4') {
        // incoming data received, process it
        processData();
        printData();
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
    rcPacket.rcBytes[(i - 7) / 2] = d + e;
    i = i + 2;
  }
}

void printData() {
  int ctr = 0;
  while (ctr<15) {
    Serial.print(rcPacket.rc[i]);
    Serial.print(",");
    ctr++;
  }
  Serial.println(rcPacket.rc[15]);
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
