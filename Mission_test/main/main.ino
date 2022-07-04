//Header file
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPSPlus.h>
#include "./rover.h"
#include "./Ultrasonic.h"
#include "./LIDAR.h"
#include "./Motor.h"
#include "./IMU.h"
#include "./communication.h"
#include "./EEPROM.h"


//-----------------------------Ultrasonic sensor--------------------------------
Ultrasonic ultrasonicHead(22,24);
Ultrasonic ultrasonicBottom(6,7);
Ultrasonic ultrasonicLong(A15);
const int emergencyStopDist = 10;

//------------------------------LIDAR sensor------------------------------
LIDAR lidar;

//------------------------------Motor------------------------------
Motor motor(8,9,11,10,12);
const int Threshold = 10;
const int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)
const int nominalSpeed = 250;
const int slowSpeed = 200;
const int verySlowSpeed = 150;

//------------------------------GPS---------------------------------
TinyGPSPlus gps;

//9axis filter
//------------------------------9axis sensor------------------------------
IMU imu(0.00, 0.00, 0.00, 0, 0, 0, 175, 20, 132);
//キャリブレーション用バッファの長さ
#define CAL_BUF_LEN 100
int bufx[CAL_BUF_LEN];
int bufy[CAL_BUF_LEN];
int calIndex = 0;

//------------------------------EEPROM------------------------------
EEPROM eeprom;

//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;

//------------------------------Onboard Camera------------------------------

//------------------------------TWElite------------------------------
TxPacketData sendData; //Packet to be coded and then written to twelite
// packetData packetTx;//Packet to be coded and then written to twelite
RxPacketData receiveData; //Received packet with GPS data
// gpsPacketUnion dataRx;//Received packet with GPS data
unsigned long start;
unsigned long stopi;

//------------------------------Rover data as global variables-----------------------
// float latA = 35.719970703125, lngA = 139.7361145019531; //目的地Aの緯度経度((教育の森公園)
// float latR = 35.715328, lngR = 139.761138;  //現在地の初期想定値(7号館屋上)
// float degRtoA; //GPS現在地における目的地の慣性方角
// float rangeRtoA;
// float x; //ローバーの慣性姿勢角
// unsigned int distanceByLIDAR = 0;


//------------------------------Control Status----------------------------
dataStruct roverData;
modeStruct roverMode = {0, 1, 0, 0};
statusStruct roverStatus = {1, 1, 0, 0, 0, 0};
successStruct roverSuccess = {0, 0, 0, 0};
boolean emergencyStopFlag = 0;

int spinsearchCount = 0;//探索中のシーケンス管理カウント
const int spinsearchIteration = 1;//探索中のスピン移動に使うループ回数

int spinmoveCount = 0;//探索後、方向に向けてスピン回数のカウント
const int spinmoveIteration = 1;//探索後のスピン移動に使うループ回数
int waitCount = 0;//探索後、スピン後停止する回数のカウント
const int waitIteration = 10;//探索後、スピン後少し停止するループ数(10よりは大きくする)
int forwardCount = 0;
const int forwardIteration = 20;//探索後、方向に向かって進むループ数


//測距用バッファの長さ
int bufdeg[MEAS_BUF_LEN];
int listdeg[SEAR_BUF_LEN];

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
  // デバッグ用シリアル通信は9600bps
  Serial.begin(115200);//ステータス設定(試験したい状況)
  roverStatus.calibration = 1;
  roverStatus.near = 0;//ゴール5m付近のとき
  roverStatus.search = 0;//ゴール5m付近で測距するとき
  //  while(1){
  //    anVolt = analogRead(HEADpin);
  //    cm_long = anVolt/2;
  //    Serial.println(cm_long);
  //    delay(1000);
  //  }
  sendData.initializeRoverComsStat();

  //超音波センサ
  ultrasonicHead.init();
  ultrasonicBottom.init();

  //モーター
  motor.init();
  motor.setThreshold(Threshold,spinThreshold);

  // Wire(Arduino-I2C)の初期化
  Wire.begin();

  // BMX055 初期化
  imu.init();

  //GPS uses Hardware Serial1
  Serial1.begin(9600);

  //TWElite uses Hardware Serial 2
  Serial2.begin(115200);
  while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(RST, OUTPUT);
  pinMode(BPS, OUTPUT);
  digitalWrite(BPS, LOW);
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);

  //LIDAR uses Hardware Serial 3
  Serial3.begin(115200);
  while (!Serial3) {
    // wait for serial port to connect. Needed for native USB port only
  }

  // EEPROM
  eeprom.init();
  
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
  while(roverMode.sleep == 1){
    //do nothing
  }
  //=================================initial Status=================================
  while (roverStatus.initial) {
    start = millis();
    if (start > (stopi + 1000)) {
      Serial.println("initial Mode: waiting for GPS...");
      stopi = millis();
    }
    if(receiveData.receiveGPS()){
      goalCalculation();//calculate distance to goals and decide root

      int first = goalRoute[0];//set first goal to the destination
      roverData.latA = receiveData.rxData.gpsData.latA[first];
      roverData.lngA = receiveData.rxData.gpsData.lngA[first];

      roverStatus.initial = 0;
      roverStatus.toGoal = 1;
    }
  }

  //=================================toGoal Status=================================


  //---------------------9軸取得--------------------------------------------------
  // BMX055 ジャイロの読み取り
  imu.getGyro();
  // BMX055 磁気の読み取り
  imu.getMag();

  //キャリブレーションが終了しているなら
  if (roverStatus.calibration == 0) {
    roverData.x = imu.angleCalculation();
  }

  //---------------------LIDARセンサ取得--------------------------------------------------
  roverData.cmLIDAR = getLIDAR();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  roverData.cmHead = ultrasonicHead.getDistance();

  //---------------------超音波(長・前面)取得--------------------------------------------------
  roverData.cmLong = ultrasonicLong.getDistance();

  //---------------------GPS acquisition--------------------------------------------------
  updateGPSlocation(&roverData.latR,&roverData.lngR);
  roverData.degRtoA = atan2((roverData.lngR - roverData.lngA) * 1.23, (roverData.latR - roverData.latA)) * 57.3 + 180;
  roverData.rangeRtoA = gps.distanceBetween(roverData.latR, roverData.lngR, roverData.latA, roverData.lngA);

  //---------------------Check parameter & update Status--------------------------------------------------
  roverData.printAll();

  if (roverData.rangeRtoA < 1.0) {
    if (roverMode.autoGpsOnly) {
      successManagement();
    }
    else if (roverMode.autoAggressive) {
      roverStatus.near = 1;
      roverStatus.search = 1;
    }
  }
  if (roverData.cmHead < emergencyStopDist) {
    emergencyStopFlag = 1;
  } else {
    emergencyStopFlag = 0;
  }

  //---------------------Special control for each status------------------------------------------------------


  //calibration
  if (roverStatus.calibration == 1) {
    calibLoop();
  }

  //ゴール探索時
  if (roverStatus.calibration == 0 && roverStatus.near == 1) {
    if (roverStatus.search == 1) { //ゴールの方向がまだ分かってない
      searchLoop();
    }
    else if (roverStatus.search == 0) { //ゴール探索時(ゴールの方向が分かって動いている時)
      //do nothing for special, just move
    }
  }

  //---------------------Motor Control--------------------------------------------------
  if (emergencyStopFlag == 1) {
    //ブレーキ
    motor.stop();
  }
  else if (emergencyStopFlag == 0 && roverStatus.calibration == 1) { //キャリブレーション時
    motor.spinLeft(slowSpeed);
    Serial.print(":calibration");
  }
  else if (emergencyStopFlag == 0 && roverStatus.calibration == 0 && roverStatus.near == 1) { //ゴール5m付近時
    if (roverStatus.search == 1) {  //ゴールの方向がまだ分かってない->スピンしながらコーンを探索中
      motor.spinRight(verySlowSpeed);
      Serial.print(":searching");
    }
    else { //ゴール探索時(ゴールの方向が分かって動いている時)
      if (forwardCount > forwardIteration) {//back to searching status after moving for a while 
        roverStatus.search = 1;
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
  else if (emergencyStopFlag == 0 && roverStatus.calibration == 0 && roverStatus.near == 0 ) { //通常走行時
    motor.angleGo(x,degRtoA,nominalSpeed);
  }
  roverData.motorControl = motor.controlStatus;

  //---------------------Logger------------------------------------------------------
  roverData.overallTime = millis();//it's good if time is synchronized with GPStime
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
    commToGS();
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
    range[i] = gps.distanceBetween(latR, lngR, receiveData.rxData.gpsData.latA[i], receiveData.rxData.gpsData.lngA[i]);
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
  if (roverStatus.toGoal < 3) {
    roverSuccess.goalGPS = roverStatus.toGoal;
    eeprom.write(27, (byte)roverSuccess.goalGPS); //logger

    int next = goalRoute[roverStatus.toGoal];//set next goal to the destination
    roverStatus.toGoal += 1;
    roverData.latA = receiveData.rxData.gpsData.latA[next];
    roverData.lngA = receiveData.rxData.gpsData.lngA[next];

    roverStatus.near = 0;
    roverStatus.search = 0;
  }
  else if (roverStatus.toGoal == 3) {
    roverSuccess.goalGPS = roverStatus.toGoal;
    roverSuccess.full = 1;
    eeprom.write(27, (byte)roverSuccess.goalGPS); //logger//logger

    roverStatus.near = 0;
    roverStatus.search = 0;
    roverMode.sleep = 1;
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
    roverStatus.calibration = 0 ;
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
    bufcm[measureIndex] = roverData.cmLidar;
    measureIndex = (measureIndex + 1) % MEAS_BUF_LEN;
    //バッファに値がたまったら
    if (measureIndex == 0) {
      //filter_angle_search();//フィルタリングした測距値をリストに一組追加する。
      listcm[searchIndex] = roverData.cmLidar;//一番最後の角度がもっともらしい。
      listdeg[searchIndex] = roverData.x;//一番最後の角度がもっともらしい。
      Serial.print(":measure deg:");
      Serial.print(listdeg[searchIndex]);
      //バッファ番号初期化(中身は放置)
      measureIndex = 0;
      spinsearchCount = 0;
      searchIndex = (searchIndex + 1) % SEAR_BUF_LEN;
      //測距リストに値がたまったら
      if (searchIndex == 0) {
        int listIndex = goal_angle_search();//リストから測距値の最小値と対応するリスト番号を探す。
        roverData.degRtoA = listdeg[listIndex];//目的地の方向を決定
        Serial.print(":searching_completed!");
        //リスト番号初期化(中身は放置)
        searchIndex = 0;
        roverStatus.search = 0;//探索終了
        emergencyStopFlag = 0;//モーター解放
      }
    }
  }
}


//=========LIDAR sensor function============================================================================//
unsigned int getLIDAR() {
  int bytenum = 0;
  while (Serial2.available() > 0)//near_flagは一時的なもの
  {
    byte c = Serial2.read();
    lidar.encode(c);
    if (lidar.distanceUpdated())
    {
      return lidar.distance();  // roverの緯度を計算
      break;
    }
  }
}

//=========GPS and position function============================================================================//

void updateGPSlocation(float* lat,float* lng) {
  while (Serial1.available() > 0)
  {
    //    Serial.print("YES");
    char c = Serial1.read();
    //    Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated())
    {
      Serial.println("");
      Serial.println("I got new GPS!");
      *lat = gps.location.lat();  // roverの緯度を計算
      *lng = gps.location.lng(); // roverの経度を計算
      break;
    }
    //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
    delay(1);
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
    dataFile.print(roverData.overallTime);
    dataFile.print(",");
    dataFile.print(imu.xMag);
    dataFile.print(",");
    dataFile.print(imu.yMag);
    dataFile.print(",");
    dataFile.print(imu.calibx);
    dataFile.print(",");
    dataFile.print(imu.caliby);
    dataFile.print(",");
    dataFile.print(roverData.x);
    dataFile.print(",");
    dataFile.print(roverData.cmHead);
    dataFile.print(",");
    dataFile.print(roverData.cmLong);
    dataFile.print(",");
    dataFile.print(roverData.cmLidar);
    dataFile.print(",");
    dataFile.print(roverData.latA);
    dataFile.print(",");
    dataFile.print(roverData.lngA);
    dataFile.print(",");
    dataFile.print(roverData.latR);
    dataFile.print(",");
    dataFile.print(roverData.lngR);
    dataFile.print(",");
    dataFile.print(roverData.degRtoA);
    dataFile.print(",");
    dataFile.print(roverData.motorControl);
    dataFile.println("");
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

}

//---------------------Communication--------------------------
void commToGS() {
  unsigned long commStart;
  unsigned long commStop;

  sendData.writeToTwelite(imu,x,distanceByLIDAR,latR,lngR,degRtoA,motor.controlStatus,overallTime);//send HK firstly
  Serial.println("Data transmission");

  commStop = millis();
  while (1) { //then go into waiting loop for ACK or NACK
    commStart = millis();
    if (commStart > commStop + 100) { //if 20ms passes, then send HK again
      sendData.writeToTwelite(imu,x,distanceByLIDAR,latR,lngR,degRtoA,motor.controlStatus,overallTime);
      Serial.println("timeout:100ms");
      break;
    }
    if (Serial2.available() > 0) {
      char c = Serial2.read();
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) {
        receiveData.buffRx[bufferPos] = c;
        bufferPos++;
      }
      else
      {
        receiveData.buffRx[bufferPos] = '\0';
        //Checks
        if (receiveData.buffRx[3] == '0' && receiveData.buffRx[4] == '1' && receiveData.buffRx[5] == '0') { //Arbitrary packet for Rover
          //Serial.println(Buffer);
          if (receiveData.buffRx[6] == '2') { //NACK
            Serial.print("NACK: Resending packet...");
            sendData.writeToTwelite(imu,x,distanceByLIDAR,latR,lngR,degRtoA,motor.controlStatus,overallTime);
          } else if (receiveData.buffRx[6] == '1') { //ACK
            Serial.print("ACK Received!");
            break;
          }
        }
        //Serial.println(buffRx);
        bufferPos = 0;
      }
    }
  }
  bufferPos = 0;
}
