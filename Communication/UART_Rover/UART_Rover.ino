// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400

//----------------- Define Structures for receiving and Handling Rover data--------------------------
typedef struct roverData{
  uint8_t roverComsStat;
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
  
typedef union packetData{
  roverData message;
  unsigned char packetData[sizeof(roverData)];
};

typedef struct gpsDataStruct{
  float latR[3];
  float longR[3];
};

typedef union gpsPacketUnion{
  gpsDataStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsPacketStruct)]
}



const int MaxBufferSize = 100;
char Buffer[MaxBufferSize];
void Parse();
int bufferPos = 0;
packetData packetTx;                                        //Packet to be coded and then written to twelite 
gpsPacketUnion dataRx;                                      //Received packet with GPS data

void  writeToTwelite();
void encodeCyclic();
uint8_t encodedTx[2*sizeof(roverData)];            //Encoded message to be sent 
uint8_t encodedRX[2*sizeof(gpsDataStruct)];
const uint8_t generator[4] = {0x46,0x23,0x17,0x0D};
const uint8_t parityCheck[3] = {0x5C,0x72,0x39};
int commsStart;
int commsStop;
int start;
int stopi;
void readRoverData();
void  writeToTwelite();
char encodedReceived[2*sizeof(roverData)];


void setup() {
  Serial.begin(115200);

  Serial.print("Twelite Rover");
  Serial.println();
  pinMode(RST, OUTPUT);
  pinMode(BPS, OUTPUT);
  digitalWrite(BPS, LOW);
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);

  Serial2.begin(115200);

}


void loop() {
  //---------------Initial Mode------------------
bool Initial = true
while (Initial){
  if (Serial2.available() > 0){
      char c = Serial2.read();
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ){
    
        Buffer[bufferPos] = c;
        /*if(bufferPos > 6 && (c!='\r'||c!='\n')){
          encodedReceived[bufferPos - 6] = c;
        }*/
        bufferPos++;
      }
      else
      {
        Buffer[bufferPos] = '\0';
        //---------------------------Checks-------------------------------------- 
        if (Buffer[3]=='0' && Buffer[4]=='1' && Buffer[5]=='0'){    //Arbitrary packet for Rover 
          //Serial.println(Buffer);
          if (Buffer[6]=='2'){                                      //NACK
            Serial.print("NACK: Resending packet...")
            writeToTwelite();
            break;
          } else if (Buffer[6]=='1'){
            break;
          } else if (Buffer[6]=='0'){
            //proccess the data
            
            Initial = false;
          }
         }
        bufferPos = 0;
      }
    }
  }



  
  start = millis();
  if (start> stopi + 5000){
  if (Serial2.available() > 0){
      char c = Serial2.read();
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ){
    
        Buffer[bufferPos] = c;
        /*if(bufferPos > 6 && (c!='\r'||c!='\n')){
          encodedReceived[bufferPos - 6] = c;
        }*/
        bufferPos++;
      }
      else
      {
        Buffer[bufferPos] = '\0';
        //---------------------------Checks-------------------------------------- 
        if (Buffer[3]=='0' && Buffer[4]=='1' && Buffer[5]=='0'){    //Arbitrary packet for Rover 
          //Serial.println(Buffer);
          if (Buffer[6]=='2'){                                      //NACK
            Serial.print("NACK: Resending packet...")
            writeToTwelite();
            break;
          } else if (Buffer[6]=='1'){
            break;
          }
         }
        
        Serial.println(Buffer);
        bufferPos = 0;
      } 
      int ctr=0;
      commsStart = millis();
      if (commsStart > commsStop + 20){      
        writeToTwelite();
        commsStop = millis();
      }
      stopi = millis();
  }     
}
}

// TWELITE write its packets in HEX then converts it to characters. The Parse function retrieves the original HEX numbers as ints.
void Parse() {
  int i = 5;
  int d,e,f;
  int checker; 

  if(Buffer[3]=='0' && Buffer[4] =='1'){
    while (i< bufferPos-1){
      checker = Buffer[i] & 0b01000000;
      //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
      if (checker == 0b01000000){
        d = Buffer[i]+9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
      }
      else{
        d = Buffer[i];
      }
      d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
      d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
      //Same for the following character
      checker = Buffer[i+1] & 0b01000000;

      if (checker == 0b01000000){
        e = Buffer[i+1] +9;
      }
      else{
        e = Buffer[i+1];
      }
      e = e & 0b00001111;
      f = d+e;
      Serial.print("0x");
      Serial.println(f,HEX);
      i = i+2;
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

void  writeToTwelite (){      
  int ctr1 = 0;
  readRoverData();
  encodeCyclic();
  Serial2.print(":000100");
  //Serial.print(":000100");
  while (ctr1<2*sizeof(Rover)){
    if((uint8_t)encodedTx[ctr1]<16){
      Serial2.print("0");
      //Serial.print("0");
    }
    Serial2.print(encodedTx[ctr1],HEX);
    //Serial.print(encodedTx[ctr1],HEX);
    ctr1++;
  }
    Serial2.print("X\r\n");
    //Serial.print("X\r\n");
}

void readRoverData(){
  packetTx.message.roverComsStat = "00000100";
  packetTx.message.xMag = 20;
  packetTx.message.yMag= 180;
  packetTx.message.calibX= 15;
  packetTx.message.calibY= 3;
  packetTx.message.x= 290.7;
  packetTx.message.cmLong = 168;
  packetTx.message.latR= 32.8;
  packetTx.message.longR= 136.75;
  packetTx.message.degRtoA= 120;
  packetTx.message.statusControl= 0b00110110;
  packetTx.message.time= 13332321;
}

void encodeCyclic() {
    uint8_t ctr = 0;
    uint8_t m;
    while(ctr<sizeof(roverMessage)) {
        m = packetTx.packetData[ctr]>>4;
        encodedTx[2*ctr] = ((m&1)*generator[3])^(((m>>1)&1)*generator[2])^
                            (((m>>2)&1)*generator[1])^(((m>>3)&1)*generator[0]);
        //Serial.print(encodedTx[2*ctr],HEX);
        m = packetTx.packetData[ctr];
        encodedTx[2*ctr+1] = ((m&1)*generator[3])^(((m>>1)&1)*generator[2])^
                            (((m>>2)&1)*generator[1])^(((m>>3)&1)*generator[0]);
        //Serial.println(encodedTx[2*ctr+1],HEX);
        ctr++;
    }
}

bool checkError(uint8_t dataByte) {
    uint8_t p[3];
    uint8_t ctr = 0;
    p[0] = dataByte&parityCheck[0];
    p[1] = dataByte&parityCheck[1];
    p[2] = dataByte&parityCheck[2];
    while(ctr<sizeof(gsMessage)/2) {
        p[0] = (p[0]&1)^(p[0]>>1);
        p[1] = (p[1]&1)^(p[1]>>1);
        p[2] = (p[2]&1)^(p[2]>>1);
        ctr++;
    }
    return (p[0]>0)||(p[1]>0)||(p[2]>0);
}
void decodeCyclic() {
    uint8_t ctr = 0;
    bool error[2];
    while (ctr<sizeof(gpsDataStruct)) {
        encodedRx[2*ctr] = encodedReceived[2*ctr]&0x7F;
        encodedRx[2*ctr+1] = encodedReceived[2*ctr+1]&0x7F;
        error[0] = checkError(encodedRx[2*ctr]);
        error[1] = checkError(encodedRx[2*ctr+1]);
        dataRX.gsPacket[ctr] = ((encodedRx[2*ctr]<<1)&0xF0)+
                                ((encodedRx[2*ctr+1]>>3)&0x0F);
                                
        if(error[0]||error[1]) { //NACK
            Serial2.print(":000102X\r\n");
            return true;
        }
        ctr++;
    }
    //If no errors send ACK
    Serial2.print(":000101X\r\n");
    return false;
}
