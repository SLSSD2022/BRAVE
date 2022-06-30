#ifndef _ROVER_HEAD_
#define _ROVER_HEAD_

#include "./IMU.h"
#include "./GPS.h"
#include "./EEPROM.h"
#include "./UltrasonicSensor.h"

typedef struct _roverDataPacket {
  uint8_t roverComsStat;
  int xMag;
  int yMag;
  uint16_t calibx;
  uint16_t caliby;
  float x;
  uint16_t cmLong;
  float latR;
  float lngR;
  float degRtoA;
  byte statusControl;
  unsigned long int time;
} roverDataPacket;

typedef struct _roverData: roverDataPacket{
  void initializeRoverComsStat();
  void updateRoverComsStat(boolean changeGoalStat, byte statusUpdate);
  void printRoverComsStat();
  void setMag(BMX055 IMU);
  void printMag();
  void setCalib(BMX055 IMU);
  void printCalib();
  void setAttitude(float x);
  void printAttitude();
  void setDistByLIDAR(uint16_t cm_LIDAR);
  void printDistByLIDAR();
  void setPosition(float latR, float lngR);
  void printPosition();
  void setDegRtoA(float degRtoA);
  void printDegRtoA();
  void setControlStatus(byte controlStatus);
  void printControlStatus();
  void setTime(unsigned long int overallTime);
  void printTime();
  void setAllData(BMX055 IMU, float x, uint16_t cm_LIDAR, float latR, float lngR, float degRtoA, byte controlStatus, unsigned long int overallTime);
  void printAllData();
} roverData;

///-----------------------------Control Status, HK-----------------------------
typedef struct _modeStruct {
  unsigned char manual : 1;
  unsigned char autoGpsOnly : 1;
  unsigned char autoAggressive : 1;
  unsigned char sleep : 1;
} modeStruct;


//Status using bit field
typedef struct _statusStruct {
  unsigned char initial : 1;
  unsigned char calibration : 1;
  unsigned int toGoal;
  unsigned char near : 1;
  unsigned char search : 1;
  unsigned char sleep : 1;
} statusStruct;


typedef struct _successStruct {
  unsigned char GPSreceive : 1;
  unsigned int goalGPS;
  unsigned int goalArrived;
  unsigned char full : 1;
} successStruct;

modeStruct roverMode = {0, 1, 0, 0};
statusStruct roverStatus = {1, 1, 0, 0, 0, 0};
successStruct roverSuccess = {0, 0, 0, 0};

#endif
