#include "./rover.h"

void Rover::initializeRoverComsStat()
{
  this->roverPacketData.message.roverComsStat = 0;
}

void Rover::updateRoverComsStat(byte statusUpdate)
{
  this->roverPacketData.message.roverComsStat = this->roverPacketData.message.roverComsStat | (uint8_t) statusUpdate;
}

void Rover::updateGoalStat()
{
  this->roverPacketData.message.roverComsStat += 4;
  if (((byte)(this->roverPacketData.message.roverComsStat) >> 2 & 0b111) == 0b110)
  {
    this->roverPacketData.message.roverComsStat += 1;
  }
}

void Rover::printRoverComsStat()
{
  Serial.print("roverComsStat:");
  Serial.println(this->roverPacketData.message.roverComsStat);
}

void Rover::setMag(bmx055 imu)
{
  this->roverPacketData.message.xMag = imu.xMag;
  this->roverPacketData.message.yMag = imu.yMag;
}

void Rover::printMag()
{
  Serial.print("xMag:");
  Serial.println(this->roverPacketData.message.xMag);
  Serial.print("yMag:");
  Serial.println(this->roverPacketData.message.yMag);
}

void Rover::setCalib(bmx055 imu)
{
  this->roverPacketData.message.calibx = imu.calibx;
  this->roverPacketData.message.caliby = imu.caliby;
}

void Rover::printCalib()
{
  Serial.print("calibx:");
  Serial.println(this->roverPacketData.message.calibx);
  Serial.print("caliby:");
  Serial.println(this->roverPacketData.message.caliby);
}

void Rover::setAttitude(float x)
{
  this->roverPacketData.message.x = x;
}

void Rover::printAttitude()
{
  Serial.print("x:");
  Serial.println(this->roverPacketData.message.x);
}
void Rover::setDistByLIDAR(uint16_t cm_LIDAR)
{
  this->roverPacketData.message.cmLong = cm_LIDAR;
}

void Rover::printDistByLIDAR()
{
  Serial.print("cm_long:");
  Serial.println(this->roverPacketData.message.cmLong);
}

void Rover::setPosition(float latR, float lngR)
{
  this->roverPacketData.message.latR = latR;
  this->roverPacketData.message.lngR = lngR;
}
void Rover::printPosition()
{
  Serial.print("latR:");
  Serial.println(this->roverPacketData.message.latR);
  Serial.print("lngR:");
  Serial.println(this->roverPacketData.message.lngR);
}
void Rover::setDegRtoA(float degRtoA)
{
  this->roverPacketData.message.degRtoA = degRtoA;
}

void Rover::printDegRtoA()
{
  Serial.print("degRtoA:");
  Serial.println(this->roverPacketData.message.degRtoA);
}

void Rover::setControlStatus(byte controlStatus)
{
  this->roverPacketData.message.statusControl = controlStatus;
}

void Rover::printControlStatus()
{
  Serial.print("controlStatus:");
  Serial.println(this->roverPacketData.message.statusControl);
}

void Rover::setTime(unsigned long int overallTime)
{
  this->roverPacketData.message.time = overallTime;
}

void Rover::printTime()
{
  Serial.print("time:");
  Serial.println(this->roverPacketData.message.time);
}

void Rover::setAllData(bmx055 imu, float x, uint16_t cm_LIDAR, float latR, float lngR, float degRtoA, byte controlStatus, unsigned long int overallTime)
{
  this->roverPacketData.message.roverComsStat = 4;
  this->setMag(imu);
  this->setCalib(imu);
  this->setAttitude(x);
  this->setDistByLIDAR(cm_LIDAR);
  this->setPosition(latR, lngR);
  this->setDegRtoA(degRtoA);
  this->setControlStatus(controlStatus);
  this->setTime(overallTime);
}

void Rover::printAllData()
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
