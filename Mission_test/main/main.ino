//Header file
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "./rover.h"
#include "./Ultrasonic.h"
#include "./LIDAR.h"
#include "./Motor.h"
#include "./GPS.h"
#include "./IMU.h"
#include "./communication.h"
#include "./EEPROM.h"


//-----------------------------Ultrasonic sensor--------------------------------
Ultrasonic ultrasonicHead(22,24);
Ultrasonic ultrasonicBottom(6,7);
Ultrasonic ultrasonicLong(A15);
const int emergencyStopDist = 10;

//------------------------------LIDAR sensor------------------------------
LIDAR lidar(&Serial1);

//------------------------------Motor------------------------------
Motor motor(3,4,5,6,7);
const int Threshold = 10;
const int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)
const int nominalSpeed = 250;
const int slowSpeed = 200;
const int verySlowSpeed = 150;

//------------------------------GPS---------------------------------
GPS gps(&Serial3);

//9axis filter
//------------------------------9axis sensor------------------------------
IMU imu(0.00, 0.00, 0.00, 0, 0, 0, 175, 20, 132);
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
Communication comm(&Serial2,2,3);//HardwareSerialPort,BPS pin,RST pin
unsigned long start;
unsigned long stopi;

//------------------------------Separation detection------------------------------
const int DETECTION_PIN = 4;

//------------------------------Control Status----------------------------
Rover rover;


boolean emergencyStopFlag = 0;

int spinsearchCount = 0;//探索中のシーケンス管理カウント
const int spinsearchIteration = 1;//探索中のスピン移動に使うループ回数

int spinmoveCount = 0;//探索後、方向に向けてスピン回数のカウント
const int spinmoveIteration = 1;//探索後のスピン移動に使うループ回数
int waitCount = 0;//探索後、スピン後停止する回数のカウント
const int waitIteration = 10;//探索後、スピン後少し停止するループ数(10よりは大きくする)
int forwardCount = 0;
const int forwardIteration = 20;//探索後、方向に向かって進むループ数


//Buffer for one range measurement near goal
#define MEAS_BUF_LEN  10//it should be more than 10, length of 9axis basic buffer
int bufcm[MEAS_BUF_LEN];
int measureIndex = 0;

//Buffer for all range measurement near goal
#define SEAR_BUF_LEN 20
int listcm[SEAR_BUF_LEN];
int searchIndex = 0;

unsigned int goalRoute[3];

int memoryFlag = 0;

/*
  ############################################################################################################
  ############################################################################################################
*/

void setup()
{
  //setting status&environment
  rover.status.calibration = 1;
  rover.status.near = 0;//ゴール5m付近のとき
  rover.status.search = 0;//ゴール5m付近で測距するとき

  //initialization
  Serial.begin(115200);// デバッグ用シリアル通信は9600bps
  ultrasonicHead.init();
  ultrasonicBottom.init();
  motor.init();motor.setThreshold(Threshold,spinThreshold);
  imu.init();
  gps.init();
  comm.init();comm.initializeRoverComsStat();
  lidar.init();
  eeprom.init(30);
  
  
  //SDcard Initialization
  pinMode(SDSW, INPUT_PULLUP);
  //  while (1) {
  //    if (digitalRead(SDSW) == 0) {
  //      Serial.println("Card inserted!");
  //      break;
  //    }
  //    else {
  //      Serial.println("Card not inserted!");
  //    }
  //    delay(1000);
  //  }
  //  if (!SD.begin(chipSelect)) {
  //    Serial.println("Card failed, or not present");
  //    // don't do anything more:
  //    while (1);
  //  }
  //  Serial.println("card initialized.");
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
  stopi = millis();
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
  while (rover.status.waitLanding) {
    start = millis();
    if (start > (stopi + 1000)) {
      Serial.println("wait for landing...");
      stopi = millis();
    }
    if(comm.waitLanding() && checkLanding()){
      rover.status.waitLanding = 0;
      rover.status.waitSeparation = 1;
      comm.updateRoverComsStat(0b10000000); //"GroundLanding" in Comms Status is 1 -> waiting for separation
      comm.sendStatus();
      stopi = millis();
    }
  }

  //==================================wait for separation status================================
  while (rover.status.waitSeparation) {
    start = millis();
    if (start > (stopi + 1000)) {
      Serial.println("wait for separation...");
      comm.sendStatus();//No updateRoverComsStat before send status -> still waiting for separation 
      stopi = millis();
    }
    if(digitalRead(DETECTION_PIN) == 1){
      comm.updateRoverComsStat(0b11000000);//"Separation Detection" in Comms Status is 1 -> waiting for distancing from MC
      comm.sendStatus(); 
      Serial.println("wait for distancing...");
      //evacuation
      motor.goStraight(nominalSpeed);
      delay(10000);
      motor.stop();
      rover.status.waitSeparation = 0;
      comm.updateRoverComsStat(0b11100000);//"Moved away/ Evacuation / Distancing" in Comms Status is 1 -> waiting for GPS
      comm.sendStatus(); 
      stopi = millis();
    }
  }


  //=================================wait for GPS status=================================
  while (rover.status.waitGPS) {
    start = millis();
    if (start > (stopi + 1000)) {
      Serial.println("initial Mode: waiting for GPS...");
      comm.sendStatus();//No updateRoverComsStat before send status -> still waiting for GPS
      stopi = millis();
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

      rover.status.waitGPS = 0;
      rover.status.toGoal = 1;
    }
  }

  //=================================toGoal Status=================================


  //---------------------9軸取得--------------------------------------------------
  // BMX055 ジャイロの読み取り
  imu.getGyro();
  // BMX055 磁気の読み取り
  imu.getMag();

  //キャリブレーションが終了しているなら
  if (rover.status.calibration == 0) {
    rover.data.x = imu.angleCalculation();
  }

  //---------------------LIDARセンサ取得--------------------------------------------------
  rover.data.cmLIDAR = lidar.getDistance();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  rover.data.cmHead = ultrasonicHead.getDistance();

  //---------------------超音波(長・前面)取得--------------------------------------------------
  rover.data.cmLong = ultrasonicLong.getDistance();

  //---------------------GPS acquisition--------------------------------------------------
  updateGPSlocation(&rover.data.latR,&rover.data.lngR);
  rover.data.degRtoA = atan2((rover.data.lngR - rover.data.lngA) * 1.23, (rover.data.latR - rover.data.latA)) * 57.3 + 180;
  rover.data.rangeRtoA = gps.distanceBetween(rover.data.latR, rover.data.lngR, rover.data.latA, rover.data.lngA);

  //---------------------Check parameter & update Status--------------------------------------------------
  imu.printAll();
  rover.data.printAll();

  if (rover.data.rangeRtoA < 1.0) {
    comm.updateGoalStat(); // Increment "aim to goal" for reaching a goal
    if (rover.mode.autoGpsOnly) {
      successManagement();
    }
    else if (rover.mode.autoAggressive) {
      rover.status.near = 1;
      rover.status.search = 1;
    }
  }
  if (rover.data.cmHead < emergencyStopDist) {
    emergencyStopFlag = 1;
  } else {
    emergencyStopFlag = 0;
  }

  //---------------------Special control for each status------------------------------------------------------


  //calibration
  if (rover.status.calibration == 1) {
    calibLoop();
  }

  //ゴール探索時
  if (rover.status.calibration == 0 && rover.status.near == 1) {
    if (rover.status.search == 1) { //ゴールの方向がまだ分かってない
      searchLoop();
    }
    else if (rover.status.search == 0) { //ゴール探索時(ゴールの方向が分かって動いている時)
      //do nothing for special, just move
    }
  }

  //---------------------Motor Control--------------------------------------------------
  if (emergencyStopFlag == 1) {
    //ブレーキ
    motor.stop();
  }
  else if (emergencyStopFlag == 0 && rover.status.calibration == 1) { //キャリブレーション時
    motor.spinLeft(slowSpeed);
    Serial.print(":calibration");
  }
  else if (emergencyStopFlag == 0 && rover.status.calibration == 0 && rover.status.near == 1) { //ゴール5m付近時
    if (rover.status.search == 1) {  //ゴールの方向がまだ分かってない->スピンしながらコーンを探索中
      motor.spinRight(verySlowSpeed);
      Serial.print(":searching");
    }
    else { //ゴール探索時(ゴールの方向が分かって動いている時)
      if (forwardCount > forwardIteration) {//back to searching status after moving for a while 
        rover.status.search = 1;
        forwardCount = 0;
        Serial.print(":restart searching");
      }
      else {
        if (spinmoveCount < spinmoveIteration) { //設定回数まで連続スピンできる
          motor_angle_spin();
        }
        else { //設定回数までスピンしたら少し停止する
          motor.stop();
          waitCount += 1;
          if (waitCount > waitIteration) {//reset counter after wait time
            waitCount = 0;
            spinmoveCount = 0;
            emergencyStopFlag = 0;
          }
        }
      }
    }
  }
  else if (emergencyStopFlag == 0 && rover.status.calibration == 0 && rover.status.near == 0 ) { //通常走行時
    motor.angleGo(x,degRtoA,nominalSpeed);
  }
  rover.data.motorControl = motor.controlStatus;

  //---------------------Logger------------------------------------------------------
  rover.data.overallTime = millis();//it's good if time is synchronized with GPStime
  LogToSDCard();
  if (memoryFlag > 5) {
    eeprom.log();
    memoryFlag = 0;
  }
  else {
    memoryFlag += 1;
  }

  //---------------------Communication(sending HK for every 10 seconds)------------------------------------------------------

  start = millis();
  int timer = start - stopi;

  Serial.print(":start");
  Serial.print(start);
  Serial.print(":stopi");
  Serial.print(stopi);
  Serial.print(":timer");
  Serial.println(timer);

  if (timer > 10000) {
    Serial.println(":Communication start!");
    comm.HKtoGS();
    stopi = millis();
    Serial.println(":Communication end!");
  }

  //---------------------ステータス更新--------------------------------------------------

  //最後にシリアル通信を改行する
  Serial.println("");
}

/*
  ############################################################################################################
  ############################################################################################################
*/



//=========Status control function============================================================================//

void goalCalculation() {
  //基本方針:最初の時点でどう巡るかを決定する。
  unsigned int range[3];
  updateGPSlocation();
  for (int i = 0; i < 3 ; i++) {
    range[i] = gps.distanceBetween(latR, lngR, comm.gpsPacket.gpsData.latA[i], comm.gpsPacket.gpsData.lngA[i]);
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

void successManagement() {
  if (rover.status.toGoal < 3) {
    rover.success.goalGPS = rover.status.toGoal;
    eeprom.write(27, (byte)rover.success.goalGPS); //logger

    int next = goalRoute[rover.status.toGoal];//set next goal to the destination
    rover.status.toGoal += 1;
    rover.data.latA = gpsPacket.gpsPacket.gpsData.latA[next];
    rover.data.lngA = gpsPacket.gpsPacket.gpsData.lngA[next];

    rover.status.near = 0;
    rover.status.search = 0;
    comm.updateGoalStat(); // Increment "aim to goal" for moving to next
  }
  else if (rover.status.toGoal == 3) {
    rover.success.goalGPS = rover.status.toGoal;
    rover.success.full = 1;
    eeprom.write(27, (byte)rover.success.goalGPS); //logger//logger

    rover.status.near = 0;
    rover.status.search = 0;
    rover.mode.sleep = 1;
  }
  return;
}

void filter_angle_search() {
  //ソート用のバッファ
  static int sortBufcm[MEAS_BUF_LEN];

  //ソート用バッファにデータをコピー
  for (int i = 0; i < MEAS_BUF_LEN; i++)
  {
    sortBufcm[i] = bufcm[i];
  }

  //クイックソートで並べ替える
  qsort(sortBufcm, MEAS_BUF_LEN, sizeof(int), quicksortFunc);

  //中央値をリストに格納する。
  listcm[searchIndex] = sortBufcm[(int)MEAS_BUF_LEN / 2];
  Serial.print(":measure cm:");
  Serial.print(listcm[searchIndex]);
}

int goal_angle_search() { //探索時、最も測距値が近い角度をゴールの方向と決定する。
  int mincm = listcm[0];
  int minindex = 0;
  for (int i = 0; i < SEAR_BUF_LEN; i++)
  {
    if (0 < listcm[i] && listcm[i] < mincm) {
      mincm = listcm[i];
      minindex = i;
    }
  }
  return minindex;
}

void calibLoop(){
  Serial.print(":xMag:");
  Serial.print(imu.xMag);
  Serial.print(":yMag:");
  Serial.print(imu.yMag);
  Serial.print(":calIndex:");
  Serial.print(calIndex);
  bufx[calIndex] = imu.xMag;
  bufy[calIndex] = imu.yMag;
  if (calIndex == CAL_BUF_LEN - 1) { //バッファに値がたまったら
    imu.calibx = xcenter_calculation();
    imu.caliby = ycenter_calculation();
    rover.status.calibration = 0 ;
    Serial.print(":calib_x:");
    Serial.print(imu.calibx);
    Serial.print(":calib_y:");
    Serial.print(imu.caliby);
    x = imu.angle_calculation();//このループ後半のためだけ
  }
  else {
    calIndex = (calIndex + 1) % CAL_BUF_LEN;
  }
  return;
}

void searchLoop(){
  if (spinsearchCount < spinsearchIteration) { //スピン段階
    emergencyStopFlag = 0;
    spinsearchCount += 1;
  }
  else {
    emergencyStopFlag = 1;//測距中は停止する
    bufcm[measureIndex] = rover.data.cmLidar;
    measureIndex = (measureIndex + 1) % MEAS_BUF_LEN;
    //バッファに値がたまったら
    if (measureIndex == 0) {
      //filter_angle_search();//フィルタリングした測距値をリストに一組追加する。
      listcm[searchIndex] = rover.data.cmLidar;//一番最後の角度がもっともらしい。
      listdeg[searchIndex] = rover.data.x;//一番最後の角度がもっともらしい。
      Serial.print(":measure deg:");
      Serial.print(listdeg[searchIndex]);
      //バッファ番号初期化(中身は放置)
      measureIndex = 0;
      spinsearchCount = 0;
      searchIndex = (searchIndex + 1) % SEAR_BUF_LEN;
      //測距リストに値がたまったら
      if (searchIndex == 0) {
        int listIndex = goal_angle_search();//リストから測距値の最小値と対応するリスト番号を探す。
        rover.data.degRtoA = listdeg[listIndex];//目的地の方向を決定
        Serial.print(":searching_completed!");
        //リスト番号初期化(中身は放置)
        searchIndex = 0;
        rover.status.search = 0;//探索終了
        emergencyStopFlag = 0;//モーター解放
      }
    }
  }
}


//=========9axis function============================================================================//


int xcenter_calculation() {
  //ソート用のバッファ
  static int sortBufx[CAL_BUF_LEN];
  //ソート用バッファにデータをコピー
  bufx[0] == bufx[CAL_BUF_LEN - 1]; //先端には0が入っちゃってるのでなんかまともな値を入れる。
  for (int i = 0; i < CAL_BUF_LEN; i++) {
    sortBufx[i] = bufx[i];
  }

  //クイックソートで並べ替える
  qsort(sortBufx, CAL_BUF_LEN, sizeof(int), quicksortFunc);

  Serial.print(":Min:");
  Serial.print(sortBufx[0]);
  Serial.print(":Max:");
  Serial.print(sortBufx[CAL_BUF_LEN - 1]);

  return (sortBufx[1] + sortBufx[CAL_BUF_LEN - 2]) / 2; //取得値ではない「0」が最少と最大になってしまう場合の対処(「0」が複数取れてしまった場合に対応できていないので注意)
}

//Medianフィルタ関数
int ycenter_calculation() {
  //ソート用のバッファ
  static int sortBufy[CAL_BUF_LEN];
  Serial.println("---------------------------------");
  bufy[0] == bufy[CAL_BUF_LEN - 1]; //先端には0が入っちゃってるのでなんかまともな値を入れる。←効果ないっぽい

  //ソート用バッファにデータをコピー
  for (int i = 0; i < CAL_BUF_LEN; i++) {
    sortBufy[i] = bufy[i];
    Serial.print(i);
    Serial.print(",");
    Serial.println(sortBufy[i]);
  }

  int k = 0;
  while (sortBufy[k] == 0) { //最初の方に値が入らなかった場合の対応←効果ないっぽい
    sortBufy[k] == sortBufy[CAL_BUF_LEN - 1];
    k += 1;
  }

  //クイックソートで並べ替える
  qsort(sortBufy, CAL_BUF_LEN, sizeof(int), quicksortFunc);

  Serial.print(":Min:");
  Serial.print(sortBufy[1]);
  Serial.print(":Max:");
  Serial.print(sortBufy[CAL_BUF_LEN - 1]);

  return (sortBufy[1] + sortBufy[CAL_BUF_LEN - 2]) / 2; //取得値ではない「0」が最少と最大になってしまう場合の対処(「0」が複数取れてしまった場合に対応できていないので注意)
}



//=========Motor Control function============================================================================//

void motor_angle_spin()
{ 
  int deltaTheta;
  if (x < degRtoA) {
    deltaTheta = degRtoA - x;
    Serial.print(":x < degRtoA:");
    Serial.print(deltaTheta);

    if ((0 <= deltaTheta && deltaTheta <= motor.spinThreshold / 2) || (360 - motor.spinThreshold / 2 <= deltaTheta && deltaTheta <= degRtoA)) {
      //閾値内にあるときは真っ直ぐ
      if ((0 <= deltaTheta && deltaTheta <= motor.threshold / 2) || (360 - motor.threshold / 2 <= deltaTheta && deltaTheta <= degRtoA)) {
        motor.goStraight(slowSpeed);
        forwardCount += 1;
      }
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else if (motor.threshold / 2 < deltaTheta && deltaTheta <= 180) {
        motor.turn(slowSpeed,slowSpeed - (deltaTheta * slowSpeed / 180))//"turn right"
      }

      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else {
        motor.turn(slowSpeed - ((360 - deltaTheta) * slowSpeed / 180),slowSpeed)//"turn left"
      }
    }
    else {
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      if (motor.threshold / 2 < deltaTheta && deltaTheta <= 180) {
        motor.spinRight(verySlowSpeed);
        spinmoveCount += 1;
      }

      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else {
        motor.spinLeft(verySlowSpeed);
        spinmoveCount += 1;
      }
    }
  }

  else {
    deltaTheta = x - degRtoA;
    Serial.print(":degRtoA < x:");
    Serial.print(deltaTheta);
    if ((0 <= deltaTheta && deltaTheta <= motor.spinThreshold / 2) || (360 - motor.spinThreshold / 2 <= deltaTheta && deltaTheta <= 360)) {
      //閾値内にあるときは真っ直ぐ
      if ((0 <= deltaTheta && deltaTheta <= motor.threshold / 2) || (360 - motor.threshold / 2 <= deltaTheta && deltaTheta <= 360)) {
        motor.goStraight(slowSpeed);
        forwardCount += 1;
      }
      //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else if (motor.threshold / 2 < deltaTheta && deltaTheta <= 180) {
        motor.turn(slowSpeed - (deltaTheta * slowSpeed / 180),slowSpeed)//"turn left"
      }

      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else {
        motor.turn(slowSpeed,slowSpeed - ((360 - deltaTheta) * slowSpeed / 180))//"turn right"
      }
    }
    else {
      if (motor.threshold / 2 < deltaTheta && deltaTheta <= 180) {
        motor.spinLeft(verySlowSpeed);
        spinmoveCount += 1;
      }
      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else {
        motor.spinRight(verySlowSpeed);
        spinmoveCount += 1;
      }
    }
  }
  Serial.print(":speedL:");
  Serial.print(motor.speedL);
  Serial.print(":speedR:");
  Serial.print(motor.speedR);
  Serial.print(":forwardCount:");
  Serial.print(forwardCount);
}

//============SDCard function=========================================================================//
void LogToSDCard() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(rover.data.overallTime);
    dataFile.print(",");
    dataFile.print(imu.xMag);
    dataFile.print(",");
    dataFile.print(imu.yMag);
    dataFile.print(",");
    dataFile.print(imu.calibx);
    dataFile.print(",");
    dataFile.print(imu.caliby);
    dataFile.print(",");
    dataFile.print(rover.data.x);
    dataFile.print(",");
    dataFile.print(rover.data.cmHead);
    dataFile.print(",");
    dataFile.print(rover.data.cmLong);
    dataFile.print(",");
    dataFile.print(rover.data.cmLidar);
    dataFile.print(",");
    dataFile.print(rover.data.latA);
    dataFile.print(",");
    dataFile.print(rover.data.lngA);
    dataFile.print(",");
    dataFile.print(rover.data.latR);
    dataFile.print(",");
    dataFile.print(rover.data.lngR);
    dataFile.print(",");
    dataFile.print(rover.data.degRtoA);
    dataFile.print(",");
    dataFile.print(rover.data.motorControl);
    dataFile.println("");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

}
