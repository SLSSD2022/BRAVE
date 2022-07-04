#include "./communication.h"

//=========TxPacketData Class============================================================================//
void TxPacketData::encodeCyclic() {
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

void  TxPacketData::writeToTwelite (IMU* imu_p,dataStruct* roverData_p) 
{
  int ctr1 = 0;
  this->setAllData(imu_p,roverData_p);
  this->printAllData();
  this->encodeCyclic();
  Serial2.print(":000100");
  //Serial.print(":000100");
  while (ctr1 < 2 * sizeof(messageStruct)) {
    if ((uint8_t)(this->encodedTx[ctr1]) < 16) {
      Serial2.print("0");
      //Serial.print("0");
    }
    Serial2.print(this->encodedTx[ctr1], HEX);
    //Serial.print(this->encodedTx[ctr1],HEX);
    ctr1++;
  }
  Serial2.print("X\r\n");
  //Serial.print("X\r\n");
}



void TxPacketData::initializeRoverComsStat()
{
  this->roverPacketData.message.roverComsStat = 0;
}

void TxPacketData::updateRoverComsStat(byte statusUpdate)
{
  this->roverPacketData.message.roverComsStat = this->roverPacketData.message.roverComsStat | (uint8_t) statusUpdate;
}

void TxPacketData::updateGoalStat()
{
  this->roverPacketData.message.roverComsStat += 4;
  if (((byte)(this->roverPacketData.message.roverComsStat) >> 2 & 0b111) == 0b110)
  {
    this->roverPacketData.message.roverComsStat += 1;
  }
}

void TxPacketData::printRoverComsStat()
{
  Serial.print("roverComsStat:");
  Serial.println(this->roverPacketData.message.roverComsStat);
}

void TxPacketData::setMag(IMU* imu_p)
{
  this->roverPacketData.message.xMag = imu_p->xMag;
  this->roverPacketData.message.yMag = imu_p->yMag;
}

void TxPacketData::printMag()
{
  Serial.print("xMag:");
  Serial.println(this->roverPacketData.message.xMag);
  Serial.print("yMag:");
  Serial.println(this->roverPacketData.message.yMag);
}

void TxPacketData::setCalib(IMU* imu_p)
{
  this->roverPacketData.message.calibx = imu_p->calibx;
  this->roverPacketData.message.caliby = imu_p->caliby;
}

void TxPacketData::printCalib()
{
  Serial.print("calibx:");
  Serial.println(this->roverPacketData.message.calibx);
  Serial.print("caliby:");
  Serial.println(this->roverPacketData.message.caliby);
}

void TxPacketData::setAttitude(float x)
{
  this->roverPacketData.message.x = x;
}

void TxPacketData::printAttitude()
{
  Serial.print("x:");
  Serial.println(this->roverPacketData.message.x);
}
void TxPacketData::setDistByLIDAR(uint16_t cm_LIDAR)
{
  this->roverPacketData.message.cmLong = cm_LIDAR;
}

void TxPacketData::printDistByLIDAR()
{
  Serial.print("cm_long:");
  Serial.println(this->roverPacketData.message.cmLong);
}

void TxPacketData::setPosition(float latR, float lngR)
{
  this->roverPacketData.message.latR = latR;
  this->roverPacketData.message.lngR = lngR;
}
void TxPacketData::printPosition()
{
  Serial.print("latR:");
  Serial.println(this->roverPacketData.message.latR);
  Serial.print("lngR:");
  Serial.println(this->roverPacketData.message.lngR);
}
void TxPacketData::setDegRtoA(float degRtoA)
{
  this->roverPacketData.message.degRtoA = degRtoA;
}

void TxPacketData::printDegRtoA()
{
  Serial.print("degRtoA:");
  Serial.println(this->roverPacketData.message.degRtoA);
}

void TxPacketData::setControlStatus(byte controlStatus)
{
  this->roverPacketData.message.motorControl = controlStatus;
}

void TxPacketData::printControlStatus()
{
  Serial.print("controlStatus:");
  Serial.println(this->roverPacketData.message.motorControl);
}

void TxPacketData::setTime(unsigned long int overallTime)
{
  this->roverPacketData.message.overallTime = overallTime;
}

void TxPacketData::printTime()
{
  Serial.print("time:");
  Serial.println(this->roverPacketData.message.overallTime);
}

void TxPacketData::setAllData(IMU* imu_p, dataStruct* roverData_p)
{
  this->roverPacketData.message.roverComsStat = 4;
  this->setMag(imu_p);
  this->setCalib(imu_p);
  this->setAttitude(roverData_p->x);
  this->setDistByLIDAR(roverData_p->cm_LIDAR);
  this->setPosition(roverData_p->latR, roverData_p->lngR);
  this->setDegRtoA(roverData_p->degRtoA);
  this->setControlStatus(roverData_p->motorControl);
  this->setTime(roverData_p->overallTime);
}

void TxPacketData::printAllData()
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

//=========RxPacketData Class============================================================================//

void RxPacketData::processData() {
  // character data is converted to uint8_t data here
  // and is stored in the this->encodedRx[] buffer
  int i = 7;
  int d, e;
  int checker;
  while (i < bufferPos - 4) {//What is lentCtr???
    checker = this->buffRx[i] & 0b01000000;
    //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
    if (checker == 0b01000000) {
      d = this->buffRx[i] + 9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
    }
    else {
      d = this->buffRx[i];
    }
    d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
    d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
    //Same for the following character
    checker = this->buffRx[i + 1] & 0b01000000;
    if (checker == 0b01000000) {
      e = this->buffRx[i + 1] + 9;
    }
    else {
      e = this->buffRx[i + 1];
    }
    e = e & 0b00001111;
    this->encodedRx[(i - 7) / 2] = d + e;
    i = i + 2;
  }
}

bool RxPacketData::checkError(uint8_t dataByte) {
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

boolean RxPacketData::decodeCyclic() {
  uint8_t ctr = 0;
  bool error[2];
  while (ctr < sizeof(gpsDataStruct)) {
    this->encodedRx[2 * ctr] = this->encodedRx[2 * ctr] & 0x7F;
    this->encodedRx[2 * ctr + 1] = this->encodedRx[2 * ctr + 1] & 0x7F;
    error[0] = this->checkError(this->encodedRx[2 * ctr]);
    error[1] = this->checkError(this->encodedRx[2 * ctr + 1]);
    this->rxData.gpsBytes[ctr] = ((this->encodedRx[2 * ctr] << 1) & 0xF0) +
                           ((this->encodedRx[2 * ctr + 1] >> 3) & 0x0F); //populate GPS data of goals in dataRx
    if (error[0] || error[1]) { //NACK
      Serial2.print(":000102X\r\n");
      return true;
    }
    ctr++;
  }
  //If no errors send ACK
  Serial2.print(":000101X\r\n");
  return false;
}

boolean RxPacketData::receiveGPS(){
  if (Serial2.available() > 0) {
    char c = Serial2.read();
    //Serial2.print(c);
    if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) { //read as data in one packet before it receives "\n"
      this->buffRx[bufferPos] = c;
      bufferPos++;
      this->buffRx[bufferPos] = '\0';
    }
    else //Check the buffa if it reads the last character in one packet
    {
      if (this->buffRx[3] == '0' && this->buffRx[4] == '1' && this->buffRx[5] == '0') { //Arbitrary packet for Rover
        if (this->buffRx[6] == '2') { //NACK
          //do nothing
        }
        else if (this->buffRx[6] == '1') { //ACK
          //do nothing
        }
        else if (this->buffRx[6] == '0') { //DATARECEIVE
          this->processData();//character data is converted to uint8_t data here and is stored in the encodedRx[] buffer
          this->decodeCyclic();//decode GPS data of three goals


          Serial.println("------------------------initial MODE SUCCESS!!!------------------------");
          Serial.print("1st GPS:");
          Serial.print(this->rxData.gpsData.latA[0]);
          Serial.print(",");
          Serial.println(this->rxData.gpsData.lngA[0]);
          Serial.print("2nd GPS:");
          Serial.print(this->rxData.gpsData.latA[1]);
          Serial.print(",");
          Serial.println(this->rxData.gpsData.lngA[1]);
          Serial.print("3rd GPS:");
          Serial.print(this->rxData.gpsData.latA[2]);
          Serial.print(",");
          Serial.println(this->rxData.gpsData.lngA[2]);
          Serial.println("---------------------------------------------------------------------");


          eeprom.logGPSdata();//log the gps data of destination to EEPROM
          Serial2.print(":000101X\r\n"); //Send ACK to MC
          return true;
        }
      }
      //Serial.println(this->buffRx);
      bufferPos = 0;
    }
  }
  return false;
}
