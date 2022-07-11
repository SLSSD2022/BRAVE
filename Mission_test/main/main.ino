//Header file
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "./Rover.h"
#include "./Ultrasonic.h"
#include "./LIDAR.h"
#include "./Motor.h"
#include "./GPS.h"
#include "./IMU.h"
#include "./Communication.h"
#include "./EEPROM.h"


//-----------------------------Ultrasonic sensor--------------------------------
Ultrasonic ultrasonicHead(22,24);//(trigpin,echopin)
Ultrasonic ultrasonicBottom(11,10);//(trigpin,echopin)
Ultrasonic ultrasonicLong(A15);//(readpin)
const int emergencyStopDist = 10;

//------------------------------LIDAR sensor------------------------------
LIDAR lidar(&Serial3);

//------------------------------Motor------------------------------
Motor motor(3,5,4,6,7);
const int Threshold = 10;
const int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)
const int nominalSpeed = 150;
const int slowSpeed = 100;
const int verySlowSpeed = 70;

//------------------------------GPS---------------------------------
GPS gps(&Serial1);

//9axis filter
//------------------------------9axis sensor------------------------------
IMU imu(0.00, 0.00, 0.00, 0, 0, 0, 265, 20, 132);
//キャリブレーション用バッファの長さ
#define CAL_BUF_LEN 100
int bufx[CAL_BUF_LEN];
int bufy[CAL_BUF_LEN];
int calIndex = 0;
//------------------------------EEPROM------------------------------
EEPROM eeprom(0x50);;//24lC1025の場合1010000(前半)or1010100(後半)を選べる

//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;

//------------------------------Onboard Camera------------------------------

//------------------------------TWElite------------------------------
Communication comm(&Serial2,8,9);//HardwareSerialPort,BPS pin,RST pin
unsigned long start;
unsigned long stop;

//------------------------------Separation detection------------------------------
const int DETECTION_PIN = 2;

//------------------------------Control Status----------------------------
Rover rover;

unsigned int goalRoute[3];

int memoryFlag = 0;

/*
  ############################################################################################################
  ############################################################################################################
*/

void setup()
{
  Serial.begin(115200);// デバッグ用シリアル通信は9600bps
  Serial.println("");
  Serial.println("===========================================================");
  Serial.println("==Hello! This is 'R2D', Relaying Rover to Destination!!!===");
  Serial.println("===========================================================");
  //setting status&environment
  rover.status.landed = 1;
  rover.status.separated = 1;
  rover.status.evacuated = 1;
  rover.status.GPSreceived = 1;
  rover.status.calibrated = 0;
  rover.status.toGoal = 1;
  rover.status.near = 0;//ゴール5m付近のとき
  rover.status.search = 0;//ゴール5m付近で測距するとき
// double LatA = 35.7100069, LngA = 139.8108103;  //目的地Aの緯度経度(スカイツリー)
//double LatA = 35.7142738, LngA = 139.76185488809645; //目的地Aの緯度経度(2号館)
//double LatA = 35.7140655517578, LngA = 139.7602539062500; //目的地Aの緯度経度(工学部広場)
//double LatA = 35.719970703125, LngA = 139.7361145019531; //目的地Aの緯度経度((教育の森公園)
//double LatR = 35.715328, LngR = 139.761138;  //現在地の初期想定値(7号館屋上)
  rover.data.latA = 35.7100069;
  rover.data.lngA = 139.8108103; //目的地Aの緯度経度(スカイツリー)
  rover.data.latR = 35.715328;
  rover.data.lngR = 139.761138;  //現在地の初期想定値(7号館屋上)
  rover.printAll();

  //initialization
  Serial.println("");
  Serial.println("------------------ Initialization!!! ------------------");
  ultrasonicHead.init();
  ultrasonicBottom.init();
  motor.init();motor.setThreshold(Threshold,spinThreshold);
  imu.init();
  gps.init();
  comm.init();comm.initializeRoverComsStat();
  lidar.init();
  eeprom.init(30);
  SDinit();
  pinMode(DETECTION_PIN,INPUT_PULLUP);


  //ログを初期化(この方法だとめっちゃ時間かかるので今後改善が必要)
  //  unsigned long k = 0;
  //  while(k < 6000){
  //    writeEEPROM(addrEEPROM, addrData,0);
  //    addrData += 1;
  //    k +=1;
  //    Serial.println(k);
  //  }
  //  addrData = 0;
  //バッファの初期化
  for (int i = 0; i < CAL_BUF_LEN; i++) {
    bufx[i] = 0;
    bufy[i] = 0;
  }
  Serial.println("------------------ Mission Start!!! ------------------");
  start = millis();
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("");
    dataFile.println("2022/7/10 18:45 simulation");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}


/*
  ############################################################################################################
  ############################################################################################################
*/


void loop()
{
  //=================================Sleep Mode=================================
  while(rover.mode.sleep){
    //do nothing
  }

  //==================================wait for landing status================================
  if (!rover.status.landed) {
    Serial.println("Rover  Initialized...");
    stop = millis();
    if (stop > (start + 1000)) {
      Serial.println("wait for landing...");
      start = millis();
    }
    if(comm.waitLanding() == 1/*&& checkLanding()*/){
      rover.status.landed = 1;
      comm.updateRoverComsStat(0b10000000); //"GroundLanding" in Comms Status is 1 -> waiting for separation
      comm.sendStatus();
      start = millis();
    }else{
      Serial.println("Communication Confirmation not received yet");
    }
  }

  //==================================wait for separation status================================
  if (!rover.status.separated) {
    stop = millis();
    if (stop > (start + 1000)) {
      Serial.println("wait for separation...");
      comm.sendStatus();//No updateRoverComsStat before send status -> still waiting for separation 
      start = millis();
    }
    if(digitalRead(DETECTION_PIN) == 0){
      comm.updateRoverComsStat(0b11000000);//"Separation Detection" in Comms Status is 1 -> waiting for distancing from MC
      comm.sendStatus(); 
      Serial.println("Pin disconnected..Start distancing...");
      //evacuation
      motor.goStraight(nominalSpeed);
      delay(10000);
      motor.stop();
      rover.status.separated = 1;
      rover.status.evacuated =1;
      comm.updateRoverComsStat(0b11100000);//"Moved away/ Evacuation / Distancing" in Comms Status is 1 -> waiting for GPS
      comm.sendStatus(); 
      start = millis();
    } else if (digitalRead(DETECTION_PIN) == 1){
      Serial.println("Pin still connected");
    }
  }


  //=================================wait for GPS status=================================
  if (!rover.status.GPSreceived) {
    stop = millis();
    if (stop > (start + 1000)) {
      Serial.println("Waiting for GPS coordinates...");
      comm.sendStatus();//No updateRoverComsStat before send status -> still waiting for GPS
      start = millis();
    }
    if(comm.receiveGPS()){
      comm.updateRoverComsStat(0b11110000); //"GPS Received" in Comms Status is 1
      comm.sendStatus();
      eeprom.logGPSdata();//log the gps data of destination to EEPROM
      goalCalculation();//calculate distance to goals and decide root

      int first = goalRoute[0];//set first goal to the destination
      comm.updateGoalStat(); //Move to next Goal
      rover.data.latA = comm.gpsPacket.gpsData.latA[first];
      rover.data.lngA = comm.gpsPacket.gpsData.lngA[first];

      rover.status.GPSreceived = 1;
      rover.status.toGoal = 1;
    }
    else{
      Serial.println("Communication Confirmation not received yet");
    }
  }

  //=================================toGoal Status=================================

  if(rover.status.toGoal > 0 && rover.success.full == 0)
  {
    toGoalLoop();
  }
}

/*
  ############################################################################################################
  ############################################################################################################
*/



//=========Status control function============================================================================//

void goalCalculation() {
  //基本方針:最初の時点でどう巡るかを決定する。
  unsigned int range[3];
  gps.updateGPSlocation(&rover.data.latR,&rover.data.lngR);
  for (int i = 0; i < 3 ; i++) {
    range[i] = gps.distanceBetween(rover.data.latR, rover.data.lngR, comm.gpsPacket.gpsData.latA[i], comm.gpsPacket.gpsData.lngA[i]);
    goalRoute[i] = i + 1;
  }
  sortRange(range, goalRoute); //root = [1,2,3]
  eeprom.write(24, (byte)goalRoute[0]);
  eeprom.write(25, (byte)goalRoute[1]);
  eeprom.write(26, (byte)goalRoute[2]);
  return;
}

void swap(unsigned int* a, unsigned int* b) {
  unsigned int temp;
  temp = *a;
  *a = *b;
  *b = temp;
  return;
}

void sortRange(unsigned int* data, unsigned int* array) {
  if (data[0] < data[1]) swap(&array[0], &array[1]);
  if (data[0] < data[2]) swap(&array[0], &array[2]);
  if (data[1] < data[2]) swap(&array[1], &array[2]);
  return;
}
