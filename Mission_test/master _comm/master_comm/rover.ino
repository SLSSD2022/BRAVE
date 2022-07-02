#include "./rover.h"

void roverData::initializeRoverComsStat()
{
  this->roverComsStat = 0;
}

void roverData::updateRoverComsStat(byte statusUpdate)
{
  this->roverComsStat = this->roverComsStat | (uint8_t) statusUpdate;
}

void roverData::updateGoalStat()
{
  this->roverComsStat += 4;
  if (((byte)(this->roverComsStat) >> 2 & 0b111) == 0b110)
  {
    this->roverComsStat += 1;
  }
}

void roverData::printRoverComsStat()
{
  Serial.print("roverComsStat:");
  Serial.println(this->roverComsStat);
}

void roverData::setMag(bmx055 imu)
{
  this->xMag = imu.xMag;
  this->yMag = imu.yMag;
}

void roverData::printMag()
{
  Serial.print("xMag:");
  Serial.println(this->xMag);
  Serial.print("yMag:");
  Serial.println(this->yMag);
}

void roverData::setCalib(bmx055 imu)
{
  this->calibx = imu.calibx;
  this->caliby = imu.caliby;
}

void roverData::printCalib()
{
  Serial.print("calibx:");
  Serial.println(this->calibx);
  Serial.print("caliby:");
  Serial.println(this->caliby);
}

void roverData::setAttitude(float x)
{
  this->x = x;
}

void roverData::printAttitude()
{
  Serial.print("x:");
  Serial.println(this->x);
}
void roverData::setDistByLIDAR(uint16_t cm_LIDAR)
{
  this->cmLong = cm_LIDAR;
}

void roverData::printDistByLIDAR()
{
  Serial.print("cm_long:");
  Serial.println(this->cmLong);
}

void roverData::setPosition(float latR, float lngR)
{
  this->latR = latR;
  this->lngR = lngR;
}
void roverData::printPosition()
{
  Serial.print("latR:");
  Serial.println(this->latR);
  Serial.print("lngR:");
  Serial.println(this->lngR);
}
void roverData::setDegRtoA(float degRtoA)
{
  this->degRtoA = degRtoA;
}

void roverData::printDegRtoA()
{
  Serial.print("degRtoA:");
  Serial.println(this->degRtoA);
}

void roverData::setControlStatus(byte controlStatus)
{
  this->statusControl = controlStatus;
}

void roverData::printControlStatus()
{
  Serial.print("controlStatus:");
  Serial.println(this->statusControl);
}

void roverData::setTime(unsigned long int overallTime)
{
  this->time = overallTime;
}

void roverData::printTime()
{
  Serial.print("time:");
  Serial.println(this->time);
}

void roverData::setAllData(bmx055 imu, float x, uint16_t cm_LIDAR, float latR, float lngR, float degRtoA, byte controlStatus, unsigned long int overallTime)
{
  this->roverComsStat = 4;
  this->setMag(imu);
  this->setCalib(imu);
  this->setAttitude(x);
  this->setDistByLIDAR(cm_LIDAR);
  this->setPosition(latR, lngR);
  this->setDegRtoA(degRtoA);
  this->setControlStatus(controlStatus);
  this->setTime(overallTime);
}

void roverData::printAllData()
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
