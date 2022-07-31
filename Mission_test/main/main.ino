//Header file
#include <Wire.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SD.h>
#include <ArduCAM.h>
#include <Servo.h>
#include "./Rover.h"
#include "./Ultrasonic.h"
#include "./LIDAR.h"
#include "./Motor.h"
#include "./GPS.h"
#include "./IMU.h"
#include "./Communication.h"
#include "./EXPROM.h"
#include "memorysaver.h"


//-----------------------------Ultrasonic sensor--------------------------------
Ultrasonic ultrasonicHead(22,24);//(trigpin,echopin)
Ultrasonic ultrasonicBottom(11,10);//(trigpin,echopin)
Ultrasonic ultrasonicLong(A15);//(readpin)
const int emergencyStopDist = 10;

//------------------------------LIDAR sensor------------------------------
LIDAR lidar(&Serial3);
// サーボモーター(ヨー)
Servo servoYaw;
const int YAW_PIN = 23;    
const int Yawdeg = 90;  
const int YawdegMax = 120;
const int YawdegMin = 85;
// サーボモーター(ピッチ)
Servo servoPitch;
const int PITCH_PIN = 25;    
const int Pitchdeg = 72;   
const int PitchdegMax = 130;
const int PitchdegMin = 30; 
//------------------------------Motor------------------------------
Motor motor(3,4,5,6,7);
const int Threshold = 10;
const int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)
const int nominalSpeed = 220;
const int slowSpeed = 150;
const int verySlowSpeed = 100;

//------------------------------GPS---------------------------------
GPS gps(&Serial1);

//9axis filter
//------------------------------9axis sensor------------------------------
IMU imu(0.00, 0.00, 0.00, 0, 0, 0, 183, -112, 71);
//キャリブレーション用バッファの長さ
#define CAL_BUF_LEN 200
int bufx[CAL_BUF_LEN];
int bufy[CAL_BUF_LEN];
int calIndex = 0;
//------------------------------EEPROM------------------------------
EXPROM eeprom(0x50);;//24lC1025の場合1010000(前半)or1010100(後半)を選べる

//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;
//#define SD_BUF_LEN 10
//dataStruct sdbuf[SD_BUF_LEN];
//int sdbufIndex = 0;
File globalFile;
unsigned long sdstart;
unsigned long sdstop;
//------------------------------Onboard Camera------------------------------
//This demo can only work on OV2640_MINI_2MP or OV5642_MINI_5MP or OV5642_MINI_5MP_BIT_ROTATION_FIXED platform.
#if !(defined OV5642_MINI_5MP || defined OV5642_MINI_5MP_BIT_ROTATION_FIXED || defined OV2640_MINI_2MP || defined OV3640_MINI_3MP)
  #error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
const int SPI_CS = 48;
#if defined (OV2640_MINI_2MP)
  ArduCAM myCAM( OV2640, SPI_CS );
#elif defined (OV3640_MINI_3MP)
  ArduCAM myCAM( OV3640, SPI_CS );
#else
  ArduCAM myCAM( OV5642, SPI_CS );
#endif


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
unsigned int tic,toc;

/*
  ############################################################################################################
  ############################################################################################################
*/

void setup()
{
  Serial.begin(115200);// デバッグ用シリアル通信は9600bps
  Serial.println("");
  Serial.println("===========================================================");
  Serial.println("==Hello! This is 'BRAVE', Relaying Rover to Destination!!!===");
  Serial.println("===========================================================");
  //setting status&environment
  rover.status.landed = 0;
  rover.status.separated = 0;
  rover.status.evacuated = 0;
  rover.status.GPSreceived = 0;
  rover.status.calibrated = 0;
  rover.status.toGoal = 0;
  rover.status.near = 0;//ゴール5m付近のとき
  rover.status.search = 0;//ゴール5m付近で測距するとき
// double LatA = 35.7100069, LngA = 139.8108103;  //目的地Aの緯度経度(スカイツリー)
//double LatA = 35.7142738, LngA = 139.76185488809645; //目的地Aの緯度経度(2号館)
//double LatA = 35.7140655517578, LngA = 139.7602539062500; //目的地Aの緯度経度(工学部広場)
//double LatA = 35.719970703125, LngA = 139.7361145019531; //目的地Aの緯度経度((教育の森公園)
//double LatR = 35.715328, LngR = 139.761138;  //現在地の初期想定値(7号館屋上)
//  rover.data.latA = 35.792549133300;
//  rover.data.lngA = 139.8909301757812; //目的地Aの緯度経度(松戸)
//  rover.data.latR = 35.792137145996;
//  rover.data.lngR = 139.8909149169921;  //現在地の初期想定値(松戸木蔭)
  rover.data.latA = 35.7429733276367;
  rover.data.lngA = 140.0115203857421; //目的地Aの緯度経度(日本大一塁)
  rover.data.latR = 35.7428932189941;
  rover.data.lngR = 140.0118103027343;  //現在地の初期想定値(日本大ホームベース)35.7428932189941,140.0118103027343

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
  ArduCAMinit();
  servoYaw.attach(YAW_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
  servoYaw.write(Yawdeg);
  servoPitch.attach(PITCH_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
  servoPitch.write(Pitchdeg);


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
  int i = EEPROM.read(0x00);
  SDprint("datalog.txt","2022/7/29 final No.");
  SDprintln("datalog.txt",i);
  
  EEPROM.write(0x00,i+1);
  Serial.println("------------------ Mission Start!!! ------------------");
  myCAMSaveToSDFile();
  start = millis();
  
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
    motor.stop();
  }

  //==================================wait for landing status================================
  while (!rover.status.landed) {
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
      myCAMSaveToSDFile();
      SDprintln("datalog.txt","Landing Confirmed");
    }
    else{
      Serial.println("Communication Confirmation not received yet");
    }
  }
  
  //==================================wait for separation status================================
  while (!rover.status.separated) {
    stop = millis();
    if (stop > (start + 1000)) {
      Serial.println("wait for separation...");
      comm.sendStatus();//No updateRoverComsStat before send status -> still waiting for separation 
      start = millis();
    }
    if(digitalRead(DETECTION_PIN) == 0){
      comm.updateRoverComsStat(0b11000000);//"Separation Detection" in Comms Status is 1 -> waiting for distancing from MC
      comm.sendStatus(); 
      myCAMSaveToSDFile();
      Serial.println("Pin disconnected..Start distancing...");
      SDprintln("datalog.txt","Separation Confirmed");
      //evacuation
      motor.goStraight(nominalSpeed);
      delay(4000);
      motor.stop();
      rover.status.separated = 1;
      rover.status.evacuated =1;
      comm.updateRoverComsStat(0b11100000);//"Moved away/ Evacuation / Distancing" in Comms Status is 1 -> waiting for GPS
      comm.sendStatus(); 
      start = millis();
      myCAMSaveToSDFile();
      SDprintln("datalog.txt","Distancing from MC Confirmed");
    } else if (digitalRead(DETECTION_PIN) == 1){
      Serial.println("Pin still connected");
    }
  }


  //=================================wait for GPS status=================================
  while (!rover.status.GPSreceived) {
    Serial.println("GPSreceiving");
    stop = millis();
    if (stop > (start + 1000)) {
      Serial.println("Waiting for GPS coordinates...");
      comm.sendStatus();//No updateRoverComsStat before send status -> still waiting for GPS
      start = millis();
    }
    if(comm.receiveGPS()){
      SDprintln("datalog.txt","Landing Confirmed");
      comm.updateRoverComsStat(0b11110000); //"GPS Received" in Comms Status is 1
      comm.sendStatus();
      
      SDlogGPSdata();
      eeprom.logGPSdata();//log the gps data of destination to EEPROM
      
      goalCalculation();//calculate distance to goals and decide root

      int first = goalRoute[0];//set first goal to the destination
      comm.updateGoalStat(); //Move to next Goal
      rover.data.latA = comm.gpsPacket.gpsData.latA[first];
      rover.data.lngA = comm.gpsPacket.gpsData.lngA[first];
      

      rover.status.GPSreceived = 1;
      rover.status.toGoal = 1;
      globalFile = SD.open("datalog.txt", FILE_WRITE);
      if (globalFile) {
      }
      else {
        Serial.println("error opening datalog.txt");
      }
    }
    else{
      Serial.println("GPS Coordinates not received yet");
    }
  }

  //=================================toGoal Status=================================

  if((rover.status.toGoal > 0)  && rover.success.full == 0)
  {
//    tic = millis();
//    Serial.println("--------------Going to Main loop...---------------");
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
  SDprint("datalog.txt","recentGPS:");
  SDprint("datalog.txt",rover.data.latR);
  SDprint("datalog.txt",",");
  SDprintln("datalog.txt",rover.data.lngR);
  
  for (int i = 0; i < 3 ; i++) {
    range[i] = gps.distanceBetween(rover.data.latR, rover.data.lngR, comm.gpsPacket.gpsData.latA[i], comm.gpsPacket.gpsData.lngA[i]);
    
    goalRoute[i] = i;
  }
  
  SDprintln("datalog.txt","--path calculated!--");
  SDprint("datalog.txt",goalRoute[0]);
  SDprint("datalog.txt",":");SDprintln("datalog.txt",range[0]);
  
  SDprint("datalog.txt",goalRoute[1]);
  SDprint("datalog.txt",":");SDprintln("datalog.txt",range[1]);
  
  SDprint("datalog.txt",goalRoute[2]);
  SDprint("datalog.txt",":");SDprintln("datalog.txt",range[2]);
  sortRange(range, goalRoute); //root = [1,2,3]
  eeprom.write(24, (byte)goalRoute[0]);
  eeprom.write(25, (byte)goalRoute[1]);
  eeprom.write(26, (byte)goalRoute[2]);
  
  SDprintln("datalog.txt","--shortest path calculated!--");
  SDprint("datalog.txt",goalRoute[0]);
  SDprint("datalog.txt",":");
  SDprint("datalog.txt",comm.gpsPacket.gpsData.latA[goalRoute[0]]);
  SDprint("datalog.txt",",");
  SDprintln("datalog.txt",comm.gpsPacket.gpsData.lngA[goalRoute[0]]);
  SDprint("datalog.txt",goalRoute[1]);
  SDprint("datalog.txt",":");
  SDprint("datalog.txt",comm.gpsPacket.gpsData.latA[goalRoute[1]]);
  SDprint("datalog.txt",",");
  SDprintln("datalog.txt",comm.gpsPacket.gpsData.lngA[goalRoute[1]]);
  SDprint("datalog.txt",goalRoute[2]);
  SDprint("datalog.txt",":");
  SDprint("datalog.txt",comm.gpsPacket.gpsData.latA[goalRoute[2]]);
  SDprint("datalog.txt",",");
  SDprintln("datalog.txt",comm.gpsPacket.gpsData.lngA[goalRoute[2]]);
  return;
}

void swapint(unsigned int* a, unsigned int* b) {
  unsigned int temp;
  temp = *a;
  *a = *b;
  *b = temp;
  return;
}

void sortRange(unsigned int* data, unsigned int* array) {
  if (data[0] > data[1]) swapint(&array[0], &array[1]);
  if (data[0] > data[2]) swapint(&array[0], &array[2]);
  if (data[1] > data[2]) swapint(&array[1], &array[2]);
  return;
}

void SDinit(){
  //SDcard Initialization
  pinMode(SDSW, INPUT_PULLUP);
  while (1) {
    if (digitalRead(SDSW) == 0) {
      Serial.println("Card inserted!");
      if (SD.begin(chipSelect)) {
        Serial.println("card initialized.");
        break;
      }
      else {
        Serial.println("Card failed, or not present");
      }
      //break;
    }
    else {
      Serial.println("Card not inserted!");
      //break;
    }
    delay(1000);
  }
  
}

void SDlogGPSdata(){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("");
    dataFile.print(comm.gpsPacket.gpsData.latA[0],5);
    dataFile.print(",");
    dataFile.println(comm.gpsPacket.gpsData.lngA[0],5);
    dataFile.print(comm.gpsPacket.gpsData.latA[1],5);
    dataFile.print(",");
    dataFile.println(comm.gpsPacket.gpsData.lngA[1],5);
    dataFile.print(comm.gpsPacket.gpsData.latA[2],5);
    dataFile.print(",");
    dataFile.println(comm.gpsPacket.gpsData.lngA[2],5);
    dataFile.println("Goal GPS Coordinates Reception Confirmed");
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}


void SDprint(String textfile,String printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.print(printdata);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}


void SDprintln(String textfile,String printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.println(printdata);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void SDprint(String textfile,int printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.print(printdata);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void SDprintln(String textfile,int printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.println(printdata);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void SDprint(String textfile,unsigned int printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.print(printdata);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void SDprintln(String textfile,unsigned int printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.println(printdata);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void SDprint(String textfile,float printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.print(printdata,5);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void SDprintln(String textfile,float printdata){
  File dataFile = SD.open(textfile, FILE_WRITE);
  if (dataFile) {
    dataFile.println(printdata,5);
    dataFile.close();
  }else {
  Serial.println("error opening datalog.txt");
  }
}

void ArduCAMinit(){
  uint8_t vid, pid;
  uint8_t temp;
  Serial.println(F("ArduCAM Start!"));
  //set the CS as an output:
  pinMode(SPI_CS,OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  // initialize SPI:
  SPI.begin();
    
  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);
    
  while(1){
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);
    
    if (temp != 0x55){
      Serial.println(F("SPI interface Error!"));
      delay(1000);continue;
    }else{
      Serial.println(F("SPI interface OK."));break;
    }
  }
  
  #if defined (OV2640_MINI_2MP)
    while(1){
      //Check if the camera module type is OV2640
      myCAM.wrSensorReg8_8(0xff, 0x01);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
      if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
        Serial.println(F("Can't find OV2640 module!"));
        delay(1000);continue;
      }
      else{
        Serial.println(F("OV2640 detected."));break;
      } 
    }
  #elif defined (OV3640_MINI_3MP)
    while(1){
      //Check if the camera module type is OV3640
      myCAM.rdSensorReg16_8(OV3640_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg16_8(OV3640_CHIPID_LOW, &pid);
      if ((vid != 0x36) || (pid != 0x4C)){
        Serial.println(F("Can't find OV3640 module!"));
        delay(1000);continue; 
      }else{
        Serial.println(F("OV3640 detected."));break;    
      }
   } 
  #else
    while(1){
      //Check if the camera module type is OV5642
      myCAM.wrSensorReg16_8(0xff, 0x01);
      myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
      if((vid != 0x56) || (pid != 0x42)){
        Serial.println(F("Can't find OV5642 module!"));
        delay(1000);continue;
      }
      else{
        Serial.println(F("OV5642 detected."));break;
      } 
    }
  #endif
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  #if defined (OV2640_MINI_2MP)
    myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  #elif defined (OV3640_MINI_3MP)
    myCAM.OV3640_set_JPEG_size(OV3640_320x240);
  #else
    myCAM.write_reg(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);   //VSYNC is active HIGH
    myCAM.OV5642_set_JPEG_size(OV5642_320x240);
  #endif
  delay(1000);
  
}
