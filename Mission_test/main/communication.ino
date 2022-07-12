  #include "./communication.h"
#include <HardwareSerial.h>

//=========Communication Class============================================================================//
Communication::Communication()
  :HWSerial(&Serial2)
  ,BPS(8)
  ,RST(9)
{
}

Communication::Communication(uint8_t bps,uint8_t rst)
  :HWSerial(&Serial2)
  ,BPS(bps)
  ,RST(rst)
{
}

Communication::Communication(HardwareSerial *serialPort,uint8_t bps,uint8_t rst)
  :HWSerial(serialPort)
  ,BPS(bps)
  ,RST(rst)
{
}

void Communication::init()
{
  //TWElite uses Hardware Serial 2
  HWSerial->begin(38400);
  while (!HWSerial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(RST, OUTPUT);
  pinMode(BPS, OUTPUT);
  digitalWrite(BPS, LOW);
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);
  Serial.println("TWElite 38400bps initialized!");
}

//=========TxPacketData function============================================================================//

void Communication::encodeCyclic() {
  uint8_t ctr = 0;
  uint8_t m;
  while (ctr < sizeof(messageStruct)) {
    m = this->roverPacketData.packetData[ctr] >> 4;
    encodedTx[2 * ctr] = ((m & 1) * this->generator[3]) ^ (((m >> 1) & 1) * this->generator[2]) ^
                         (((m >> 2) & 1) *this->generator[1]) ^ (((m >> 3) & 1) * this->generator[0]);
    //Serial.print(this->encodedTx[2*ctr],HEX);
    m = this->roverPacketData.packetData[ctr];
    encodedTx[2 * ctr + 1] = ((m & 1) * this->generator[3]) ^ (((m >> 1) & 1) * this->generator[2]) ^
                             (((m >> 2) & 1) * this->generator[1]) ^ (((m >> 3) & 1) * this->generator[0]);
    //Serial.println(this->encodedTx[2*ctr+1],HEX);
    ctr++;
  }
}

void Communication::encodeCyclicStatus() {
  uint8_t ctr = 0;
  uint8_t m;
  // While not necessary, only one byte, check with Charleston

  m = this->roverPacketData.packetData[0] >> 4;
  statusTx[2 * ctr] = ((m & 1) * this->generator[3]) ^ (((m >> 1) & 1) * this->generator[2]) ^
                        (((m >> 2) & 1) *this->generator[1]) ^ (((m >> 3) & 1) * this->generator[0]);
  
  m = this->roverPacketData.packetData[0];
  statusTx[2 * ctr + 1] = ((m & 1) * this->generator[3]) ^ (((m >> 1) & 1) * this->generator[2]) ^
                            (((m >> 2) & 1) * this->generator[1]) ^ (((m >> 3) & 1) * this->generator[0]);

}

void  Communication::writeToTwelite (IMU* imu_p,dataStruct* roverData_p) 
{
  int ctr1 = 0;
  this->setAllData(imu_p,roverData_p);
  this->printAllData();
  this->encodeCyclic();
  HWSerial->print(":000100");
  //Serial.print(":000100");
  while (ctr1 < 2 * sizeof(messageStruct)) {
    if ((uint8_t)(this->encodedTx[ctr1]) < 16) {
      HWSerial->print("0");
      //Serial.print("0");
    }
    HWSerial->print(this->encodedTx[ctr1], HEX);
    //Serial.print(this->encodedTx[ctr1],HEX);
    ctr1++;
  }
  HWSerial->print("X\r\n");
  //Serial.print("X\r\n");
}

void Communication::HKtoGS(IMU* imu_p,dataStruct* roverData_p)
{
  unsigned long commStart;
  unsigned long commStop;

  writeToTwelite(imu_p,roverData_p);//send HK firstly
  Serial.println("Data transmission");

  commStop = millis();
  while (1) { //then go into waiting loop for ACK or NACK
    commStart = millis();
    if (commStart > commStop + 100) { //if 20ms passes, then send HK again
      writeToTwelite(imu_p,roverData_p);
      Serial.println("timeout:100ms");
      break;
    }
    if (HWSerial->available() > 0) {
      char c = HWSerial->read();
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) {
        buff[bufferPos] = c;
        bufferPos++;
      }
      else
      {
        buff[bufferPos] = '\0';
        //Checks
        if (buff[3] == '0' && buff[4] == '1' && buff[5] == '0') { //Arbitrary packet for Rover
          //Serial.println(Buffer);
          if (buff[6] == '2') { //NACK
            Serial.print("NACK: Resending packet...");
            writeToTwelite(imu_p,roverData_p);
          } else if (buff[6] == '1') { //ACK
            Serial.print("ACK Received!");
            break;
          }
        }
        //Serial.println(buff);
        bufferPos = 0;
      }
    }
  }
  bufferPos = 0;
}

void Communication::sendStatus()
{
  int ctr2 = 0;
  this->encodeCyclicStatus();
  HWSerial->print(":000100");
  //Serial.print(":000100");
  while (ctr2 < 2 * sizeof(messageStruct::roverComsStat)) {
    if ((uint8_t)(this->statusTx[ctr2]) < 16) {
      HWSerial->print("0");
      //Serial.print("0");
    }
    HWSerial->print(this->statusTx[ctr2], HEX);
    //Serial.print(this->encodedTx[ctr2],HEX);
    ctr2++;
  }
  HWSerial->print("X\r\n");
  //Serial.print("X\r\n");
}

//=========update function============================================================================//

void Communication::initializeRoverComsStat()
{
  this->roverPacketData.message.roverComsStat = 0;
}

void Communication::updateRoverComsStat(byte statusUpdate)
{
  this->roverPacketData.message.roverComsStat = this->roverPacketData.message.roverComsStat | (uint8_t) statusUpdate;
}

void Communication::updateGoalStat()
{
  this->roverPacketData.message.roverComsStat += 0b10;
  if (((byte)(this->roverPacketData.message.roverComsStat) >> 1 & 0b111) == 0b110)
  {
    this->roverPacketData.message.roverComsStat += 0b01;
  }
}

void Communication::printRoverComsStat()
{
  Serial.print("roverComsStat:");
  Serial.println(this->roverPacketData.message.roverComsStat);
}

void Communication::setMag(IMU* imu_p)
{
  this->roverPacketData.message.xMag = imu_p->xMag;
  this->roverPacketData.message.yMag = imu_p->yMag;
}

void Communication::printMag()
{
  Serial.print("xMag:");
  Serial.println(this->roverPacketData.message.xMag);
  Serial.print("yMag:");
  Serial.println(this->roverPacketData.message.yMag);
}

void Communication::setCalib(IMU* imu_p)
{
  this->roverPacketData.message.calibx = imu_p->calibx;
  this->roverPacketData.message.caliby = imu_p->caliby;
}

void Communication::printCalib()
{
  Serial.print("calibx:");
  Serial.println(this->roverPacketData.message.calibx);
  Serial.print("caliby:");
  Serial.println(this->roverPacketData.message.caliby);
}

void Communication::setAttitude(float x)
{
  this->roverPacketData.message.x = x;
}

void Communication::printAttitude()
{
  Serial.print("x:");
  Serial.println(this->roverPacketData.message.x);
}

void Communication::setDistBottom(uint16_t cm_Bottom)
{
  this->roverPacketData.message.cmBottom = cm_Bottom;
}

void Communication::setDistHead(uint16_t cm_Head)
{
  this->roverPacketData.message.cmHead = cm_Head;
}

void Communication::setDistLong(uint16_t cm_Long)
{
  this->roverPacketData.message.cmLong = cm_Long;
}

void Communication::setDistLIDAR(uint16_t cm_Lidar)
{
  this->roverPacketData.message.cmLidar = cm_Lidar;
}

void Communication::printDistByLIDAR()
{
  Serial.print("cm_long:");
  Serial.println(this->roverPacketData.message.cmLong);
}

void Communication::setGoalPosition(float latA, float lngA)
{
  this->roverPacketData.message.latA = latA;
  this->roverPacketData.message.lngA = lngA;
}

void Communication::setPosition(float latR, float lngR)
{
  this->roverPacketData.message.latR = latR;
  this->roverPacketData.message.lngR = lngR;
}

void Communication::printPosition()
{
  Serial.print("latR:");
  Serial.println(this->roverPacketData.message.latR);
  Serial.print("lngR:");
  Serial.println(this->roverPacketData.message.lngR);
}
void Communication::setDegRtoA(float degRtoA)
{
  this->roverPacketData.message.degRtoA = degRtoA;
}

void Communication::printDegRtoA()
{
  Serial.print("degRtoA:");
  Serial.println(this->roverPacketData.message.degRtoA);
}

void Communication::setRangeRtoA(float rangeRtoA)
{
  this->roverPacketData.message.rangeRtoA = rangeRtoA;
}

void Communication::setControlStatus(byte controlStatus)
{
  this->roverPacketData.message.motorControl = controlStatus;
}

void Communication::printControlStatus()
{
  Serial.print("controlStatus:");
  Serial.println(this->roverPacketData.message.motorControl);
}

void Communication::setTime(unsigned long int overallTime)
{
  this->roverPacketData.message.overallTime = overallTime;
}

void Communication::printTime()
{
  Serial.print("time:");
  Serial.println(this->roverPacketData.message.overallTime);
}

void Communication::setAllData(IMU* imu_p, dataStruct* roverData_p)
{
  //this->roverPacketData.message.roverComsStat = 0; -> if we have updateComStat and updateGoalStat functions this is not necessary
  this->setMag(imu_p);
  this->setCalib(imu_p);
  this->setAttitude(roverData_p->x);
  this->setDistBottom(roverData_p->cmBottom);
  this->setDistHead(roverData_p->cmHead);
  this->setDistLong(roverData_p->cmLong);
  this->setDistLIDAR(roverData_p->cmLidar);
  this->setGoalPosition(roverData_p->latA, roverData_p->lngA);
  this->setPosition(roverData_p->latR, roverData_p->lngR);
  this->setDegRtoA(roverData_p->degRtoA);
  this->setRangeRtoA(roverData_p->rangeRtoA);
  this->setControlStatus(roverData_p->motorControl);
  this->setTime(roverData_p->overallTime);
}

void Communication::printAllData()
{
  this->printRoverComsStat();
  this->printMag();
  this->printCalib();
  this->printAttitude();
  this->printDistByLIDAR();
  this->printPosition();
  this->printDegRtoA();
  this->printControlStatus();
  this->printTime();
}

//=========Receive function============================================================================//
void Communication::processData() {
  // character data is converted to uint8_t data here
  // and is stored in the this->encodedRx[] buffer
  int i = 7;
  int d, e;
  int checker;
  while (i < bufferPos - 4) {//What is lentCtr???
    checker = this->buff[i] & 0b01000000;
    //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
    if (checker == 0b01000000) {
      d = this->buff[i] + 9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
    }
    else {
      d = this->buff[i];
    }
    d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
    d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
    //Same for the following character
    checker = this->buff[i + 1] & 0b01000000;
    if (checker == 0b01000000) {
      e = this->buff[i + 1] + 9;
    }
    else {
      e = this->buff[i + 1];
    }
    e = e & 0b00001111;
    this->encodedRx[(i - 7) / 2] = d + e;
    i = i + 2;
  }
}

bool Communication::checkError(uint8_t dataByte) {
  uint8_t p[3];
  uint8_t ctr = 0;
  p[0] = dataByte & this->parityCheck[0];
  p[1] = dataByte & this->parityCheck[1];
  p[2] = dataByte & this->parityCheck[2];
  while (ctr < sizeof(gpsDataStruct) / 2) {
    p[0] = (p[0] & 1) ^ (p[0] >> 1);
    p[1] = (p[1] & 1) ^ (p[1] >> 1);
    p[2] = (p[2] & 1) ^ (p[2] >> 1);
    ctr++;
  }
  return (p[0] > 0) || (p[1] > 0) || (p[2] > 0);
}


boolean Communication::decodeCyclic() {
  uint8_t ctr = 0;
  bool error[2];
  while (ctr < sizeof(gpsDataStruct)) {
    this->encodedRx[2 * ctr] = this->encodedRx[2 * ctr] & 0x7F;
    this->encodedRx[2 * ctr + 1] = this->encodedRx[2 * ctr + 1] & 0x7F;
    error[0] = this->checkError(this->encodedRx[2 * ctr]);
    error[1] = this->checkError(this->encodedRx[2 * ctr + 1]);
    this->gpsPacket.gpsBytes[ctr] = ((this->encodedRx[2 * ctr] << 1) & 0xF0) +
                           ((this->encodedRx[2 * ctr + 1] >> 3) & 0x0F); //populate GPS data of goals in dataRx
    if (error[0] || error[1]) { //NACK
      HWSerial->print(":000102X\r\n");
      return true;
    }
    ctr++;
  }
  //If no errors send ACK
  HWSerial->print(":000101X\r\n");
  return false;
}




boolean Communication::receiveGPS(){
  while (1){
  if (HWSerial->available() > 0) {
    char c = HWSerial->read();
    //HWSerial->print(c);
    if ( c != '\n' && (this->bufferPos < MaxBufferSize - 1) ) { //read as data in one packet before it receives "\n"
      buff[this->bufferPos] = c;
      this->bufferPos++;
      buff[this->bufferPos] = '\0';
    }
    else //Check the buffa if it reads the last character in one packet
    {
      if (buff[3] == '0' && buff[4] == '1' && buff[5] == '0') { //Arbitrary packet for Rover
        if (buff[6] == '2') { //NACK
          //do nothing
        }
        else if (buff[6] == '1') { //ACK
          //do nothing
        }
        else if (buff[6] == '0') { //DATARECEIVE
          this->processData();//character data is converted to uint8_t data here and is stored in the encodedRx[] buffer
          this->decodeCyclic();//decode GPS data of three goals


          Serial.println("------------------------initial MODE SUCCESS!!!------------------------");
          Serial.print("1st GPS:");
          Serial.print(this->gpsPacket.gpsData.latA[0]);
          Serial.print(",");
          Serial.println(this->gpsPacket.gpsData.lngA[0]);
          Serial.print("2nd GPS:");
          Serial.print(this->gpsPacket.gpsData.latA[1]);
          Serial.print(",");
          Serial.println(this->gpsPacket.gpsData.lngA[1]);
          Serial.print("3rd GPS:");
          Serial.print(this->gpsPacket.gpsData.latA[2]);
          Serial.print(",");
          Serial.println(this->gpsPacket.gpsData.lngA[2]);
          Serial.println("---------------------------------------------------------------------");

          HWSerial->print(":000101X\r\n"); //Send ACK to MC
          return true;
        }
      }
      //Serial.println(buff);
      this->bufferPos = 0;
      return false;
    }
  }
 }
}

boolean Communication::waitLanding(){
  while (1){
    if (HWSerial->available() > 0) {
      char c = HWSerial->read();
      if ( c != '\n' && (this->bufferPos < MaxBufferSize - 1) ) { //read as data in one packet before it receives "\n"
        buff[this->bufferPos] = c;
        this->bufferPos++;
        buff[this->bufferPos] = '\0';
      }
      else //Check the buffa if it reads the last character in one packet
      {
        if (buff[3] == '0' && buff[4] == '1' && buff[5] == '0') { //Arbitrary packet for Rover
          if (buff[6] == '2') { //NACK
            //do nothing
          }
          else if (buff[6] == '1') { //ACK
            //do nothing
          }
          else if (buff[6] == '3') { //DATARECEIVE
            Serial.println("------------------------Multicopter Landing Signal Received!------------------------");
  
            HWSerial->print(":000101X\r\n"); //Send ACK to MC
            Serial.println(buff);
            Serial.println("ACK sent");
            return true;
          }
        }
        //Serial.println(buff);
        this->bufferPos = 0;
        return false;
      }
    }
  }
}
