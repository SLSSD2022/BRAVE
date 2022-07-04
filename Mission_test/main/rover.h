#ifndef _ROVER_HEAD_
#define _ROVER_HEAD_

#include "./IMU.h"
#include "./GPS.h"
#include "./EEPROM.h"
#include "./Ultrasonic.h"

typedef struct _dataStruct {
  uint8_t roverComsStat;
  float x;
  uint16_t cmBottom;
  uint16_t cmHead;
  uint16_t cmLong;
  uint16_t cmLidar;
  float latA;
  float lngA;
  float latR;
  float lngR;
  float degRtoA;
  float rangeRtoA;
  byte motorControl;
  unsigned long int overallTime;
  void printAll();
} dataStruct;


typedef struct _modeStruct {
  unsigned char manual : 1;
  unsigned char autoGpsOnly : 1;
  unsigned char autoAggressive : 1;
  unsigned char sleep : 1;
} modeStruct;


typedef struct _statusStruct {//Status using bit field
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

#endif
