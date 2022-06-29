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
  void setMag(int xMag, int yMag);
  void printMag();
  void setCalib(uint16_t calibx, uint16_t caliby);
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
  void setAllData(int xMag, int yMag, uint16_t calibx, uint16_t caliby, float x, uint16_t cm_LIDAR, float latR, float lngR, float degRtoA, byte controlStatus, unsigned long int overallTime);
  void printAllData();
} roverData;
