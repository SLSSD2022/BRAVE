#include "./rover.h"

void RoverData::initializeRoverComsStat()
{
  this->roverPacketData.message.roverComsStat = 0;
}

void RoverData::updateRoverComsStat(byte statusUpdate)
{
  this->roverPacketData.message.roverComsStat = this->roverPacketData.message.roverComsStat | (uint8_t) statusUpdate;
}

void RoverData::updateGoalStat()
{
  this->roverPacketData.message.roverComsStat += 4;
  if (((byte)(this->roverPacketData.message.roverComsStat) >> 2 & 0b111) == 0b110)
  {
    this->roverPacketData.message.roverComsStat += 1;
  }
}

void RoverData::printRoverComsStat()
{
  Serial.print("roverComsStat:");
  Serial.println(this->roverPacketData.message.roverComsStat);
}

void RoverData::setMag(bmx055 imu)
{
  this->roverPacketData.message.xMag = imu.xMag;
  this->roverPacketData.message.yMag = imu.yMag;
}

void RoverData::printMag()
{
  Serial.print("xMag:");
  Serial.println(this->roverPacketData.message.xMag);
  Serial.print("yMag:");
  Serial.println(this->roverPacketData.message.yMag);
}

void RoverData::setCalib(bmx055 imu)
{
  this->roverPacketData.message.calibx = imu.calibx;
  this->roverPacketData.message.caliby = imu.caliby;
}

void RoverData::printCalib()
{
  Serial.print("calibx:");
  Serial.println(this->roverPacketData.message.calibx);
  Serial.print("caliby:");
  Serial.println(this->roverPacketData.message.caliby);
}

void RoverData::setAttitude(float x)
{
  this->roverPacketData.message.x = x;
}

void RoverData::printAttitude()
{
  Serial.print("x:");
  Serial.println(this->roverPacketData.message.x);
}
void RoverData::setDistByLIDAR(uint16_t cm_LIDAR)
{
  this->roverPacketData.message.cmLong = cm_LIDAR;
}

void RoverData::printDistByLIDAR()
{
  Serial.print("cm_long:");
  Serial.println(this->roverPacketData.message.cmLong);
}

void RoverData::setPosition(float latR, float lngR)
{
  this->roverPacketData.message.latR = latR;
  this->roverPacketData.message.lngR = lngR;
}
void RoverData::printPosition()
{
  Serial.print("latR:");
  Serial.println(this->roverPacketData.message.latR);
  Serial.print("lngR:");
  Serial.println(this->roverPacketData.message.lngR);
}
void RoverData::setDegRtoA(float degRtoA)
{
  this->roverPacketData.message.degRtoA = degRtoA;
}

void RoverData::printDegRtoA()
{
  Serial.print("degRtoA:");
  Serial.println(this->roverPacketData.message.degRtoA);
}

void RoverData::setControlStatus(byte controlStatus)
{
  this->roverPacketData.message.statusControl = controlStatus;
}

void RoverData::printControlStatus()
{
  Serial.print("controlStatus:");
  Serial.println(this->roverPacketData.message.statusControl);
}

void RoverData::setTime(unsigned long int overallTime)
{
  this->roverPacketData.message.time = overallTime;
}

void RoverData::printTime()
{
  Serial.print("time:");
  Serial.println(this->roverPacketData.message.time);
}

void RoverData::setAllData(bmx055 imu, float x, uint16_t cm_LIDAR, float latR, float lngR, float degRtoA, byte controlStatus, unsigned long int overallTime)
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

void RoverData::printAllData()
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
