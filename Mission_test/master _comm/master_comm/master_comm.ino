//Header file
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "./rover.h"
#include "./communication.h"
#include "./EEPROM.h"
//------------------------------LIDAR sensor------------------------------
//LIDAR
unsigned int cm_LIDAR = 0;
unsigned int LIDAR_buf = 0;

//------------------------------Motor------------------------------
const int ENABLE = 8;
const int CH1 = 9;
const int CH2 = 11;
const int CH3 = 10;
const int CH4 = 12;

int nominalSpeed = 250;
int slowSpeed = 200;
int verySlowSpeed = 150;
int speedR;
int speedL;
int forwardCount = 0;

float x; //ローバーの慣性姿勢角
float deltaTheta;//目的方向と姿勢の相対角度差
int threshold = 10; //角度の差分の閾値
int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)

EEPROM eeprom;

//9axis filter
//姿勢フィルターバッファの長さ
#define BUF_LEN 10
int buf[BUF_LEN];
int index = 0;
float filterVal = 0.0; //フィルター後の値
//キャリブレーション用バッファの長さ
#define CAL_BUF_LEN 100
int bufx[CAL_BUF_LEN];
int bufy[CAL_BUF_LEN];
int calIndex = 0;
//測距用バッファの長さ
int bufdeg[MEAS_BUF_LEN];
int listdeg[SEAR_BUF_LEN];

//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;


//------------------------------Onboard Camera------------------------------


boolean stopFlag = 0;

int searchCount = 0;//探索中のシーケンス管理カウント
const int spinIteration = 1;//探索中のスピン移動に使うループ回数
const int spinmove_iteration = 1;//探索後のスピン移動に使うループ回数

int spinCount = 0;//探索後、方向に向けてスピン回数のカウント
int waitSpin = 0;//探索後、スピン後停止する回数のカウント
const int waitIteration = 10;//探索後、スピン後少し停止するループ数(10よりは大きくする)
const int forward_iteration = 20;//探索後、方向に向かって進むループ数

unsigned int goalRoute[3];

int memoryFlag = 0;
int controlStatus;
unsigned long overallTime;


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
  sendData.txPacket.message.initializeRoverComsStat();

  //超音波センサ
  pinMode(HEAD_Trig, OUTPUT);
  pinMode(HEAD_Echo, INPUT);

  //モーター
  pinMode(CH1, OUTPUT);
  pinMode(CH2, OUTPUT);
  pinMode(CH3, OUTPUT);
  pinMode(CH4, OUTPUT);
  digitalWrite(ENABLE, LOW); // disable

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
    ; // wait for serial port to connect. Needed for native USB port only
  }

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


  //初期値
  degRtoA = atan2((lngR - LngA) * 1.23, (latR - LatA)) * 57.3 + 180;
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
  //=================================initial Mode=================================
  while (roverStatus.initial) {
    start = millis();
    if (start > stopi + 1000) {
      Serial.println("initial Mode: waiting for GPS...");
      stopi = millis();
    }
    if(receiveData.receiveGPS()){
      goalCalculation();//calculate distance to goals and decide root

      int first = goalRoute[0];//set first goal to the destination
      LatA = receiveData.rxData.gpsData.latA[first];
      LngA = receiveData.rxData.gpsData.lngA[first];

      roverStatus.initial = 0;
      roverStatus.toGoal = 1;
    }
  }

  //=================================Nominal Mode=================================


  //---------------------9軸取得--------------------------------------------------
  // BMX055 ジャイロの読み取り
  imu.getGyro();
  // BMX055 磁気の読み取り
  imu.getMag();

  //キャリブレーションが終了しているなら
  if (roverStatus.calibration == 0) {
    x = angle_calculation();
  }

  //---------------------LIDARセンサ取得--------------------------------------------------
  cm_LIDAR = getLIDAR();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  obstacleDistance = getUltrasonic_HEAD();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  anVolt = analogRead(HEADpin);
  cm_long = anVolt / 2;

  //---------------------GPS acquisition--------------------------------------------------
  updateGPSlocation();
  degRtoA = atan2((lngR - LngA) * 1.23, (latR - LatA)) * 57.3 + 180;
  rangeRtoA = gps.distanceBetween(latR, lngR, LatA, LngA);

  //---------------------Check parameter & update Status--------------------------------------------------

  Serial.print(":latR:");
  Serial.print(latR);
  Serial.print(":lngR:");
  Serial.print(lngR);
  Serial.print(":LatA:");
  Serial.print(LatA);
  Serial.print(":LngA:");
  Serial.print(LngA);
  Serial.print(":degRtoA:");
  Serial.print(degRtoA);
  Serial.print(":x:");
  Serial.print(x);
  Serial.print(":rangeRtoA:");
  Serial.print(rangeRtoA);
  if (rangeRtoA < 1.0) {
    if (roverMode.autoGpsOnly) {
      successManagement();
    }
    else if (roverMode.autoAggressive) {
      roverStatus.near = 1;
      roverStatus.search = 1;
    }
  }
  Serial.print(":cm_LIDAR:");
  Serial.print(cm_LIDAR);
  if (obstacleDistance < emergencyStopDist) {
    stopFlag = 1;
  } else {
    stopFlag = 0;
  }
  if (roverMode.sleep == 1) {
    stopFlag = 1;
  }

  //---------------------Special control for each status------------------------------------------------------


  //calibration
  if (roverStatus.calibration == 1) {
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
      Serial.print(":Calib_x:");
      Serial.print(imu.calibx);
      Serial.print(":Calib_y:");
      Serial.print(imu.caliby);
      x = angle_calculation();//このループ後半のためだけ
    }
    else {
      calIndex = (calIndex + 1) % CAL_BUF_LEN;
    }
  }

  //ゴール探索時
  if (roverStatus.calibration == 0 && roverStatus.near == 1) {
    if (roverStatus.search == 1) { //ゴールの方向がまだ分かってない
      if (searchCount < spinIteration) { //スピン段階
        stopFlag = 0;
        searchCount += 1;
      }
      else {
        stopFlag = 1;//測距中は停止する
        bufcm[measureIndex] = cm_LIDAR;
        measureIndex = (measureIndex + 1) % MEAS_BUF_LEN;
        //バッファに値がたまったら
        if (measureIndex == 0) {
          //filter_angle_search();//フィルタリングした測距値をリストに一組追加する。
          listcm[searchIndex] = cm_LIDAR;//一番最後の角度がもっともらしい。
          listdeg[searchIndex] = x;//一番最後の角度がもっともらしい。
          Serial.print(":measure deg:");
          Serial.print(listdeg[searchIndex]);
          //バッファ番号初期化(中身は放置)
          measureIndex = 0;
          searchCount = 0;
          searchIndex = (searchIndex + 1) % SEAR_BUF_LEN;
          //測距リストに値がたまったら
          if (searchIndex == 0) {
            int listIndex = goal_angle_search();//リストから測距値の最小値と対応するリスト番号を探す。
            degRtoA = listdeg[listIndex];//目的地の方向を決定
            Serial.print(":searching_completed!");
            //リスト番号初期化(中身は放置)
            searchIndex = 0;
            roverStatus.search = 0;//探索終了
            stopFlag = 0;//モーター解放
          }
        }
      }
    }
    else if (roverStatus.search == 0) { //ゴール探索時(ゴールの方向が分かって動いている時)
      if (spinCount < spinIteration) { //設定回数まで連続スピンできる
        stopFlag = 0;
      }
      else { //設定回数までスピンしたら少し停止する
        stopFlag = 1;
        waitSpin += 1;
        if (waitSpin > waitIteration) {
          waitSpin = 0;
          spinCount = 0;
          stopFlag = 0;
        }
      }
    }
  }

  //---------------------Motor Control--------------------------------------------------
  if (stopFlag == 1) {
    //ブレーキ
    digitalWrite(ENABLE, HIGH);
    digitalWrite(CH1, HIGH);
    digitalWrite(CH2, HIGH);
    digitalWrite(CH3, HIGH);
    digitalWrite(CH4, HIGH);
    controlStatus = 0;//"stop"
    Serial.print(":stop!");
  }
  else if (stopFlag == 0 && roverStatus.calibration == 1) { //キャリブレーション時
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, slowSpeed);
    analogWrite(CH2, 0);
    analogWrite(CH3, 0);
    analogWrite(CH4, slowSpeed);
    controlStatus = 7;//"calibration..."
    Serial.print(":calibration");
  }
  else if (stopFlag == 0 && roverStatus.calibration == 0 && roverStatus.near == 1) { //ゴール5m付近時
    if (roverStatus.search == 1) { //スピンしながらコーンを探索
      digitalWrite(ENABLE, HIGH); // enable
      analogWrite(CH1, 0);
      analogWrite(CH2, verySlowSpeed);
      analogWrite(CH3, verySlowSpeed);
      analogWrite(CH4, 0);
      controlStatus = 8;//"searching..."
      Serial.print(":searching");
    }
    else { //コーンの方を向く
      if (forwardCount > forward_iteration) {
        roverStatus.search = 1;
        forwardCount = 0;
        Serial.print(":restart searching");
      }
      else {
        motor_angle_spin();
      }
    }
  }
  else if (stopFlag == 0 && roverStatus.calibration == 0 && roverStatus.near == 0 ) { //通常走行時
    motor_angle_go();
  }


  //---------------------Logger------------------------------------------------------
  //  LogToSDCard();
  if (memoryFlag > 5) {
    eeprom.Log();
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

  if ( timer > 10000) {
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
  eeprom.write(eeprom.addrEEPROM, 24, (byte)goalRoute[0]);
  eeprom.write(eeprom.addrEEPROM, 25, (byte)goalRoute[1]);
  eeprom.write(eeprom.addrEEPROM, 26, (byte)goalRoute[2]);
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
    eeprom.write(eeprom.addrEEPROM, 27, (byte)roverSuccess.goalGPS); //logger

    int next = goalRoute[roverStatus.toGoal];//set next goal to the destination
    roverStatus.toGoal += 1;
    LatA = receiveData.rxData.gpsData.latA[next];
    LngA = receiveData.rxData.gpsData.lngA[next];

    roverStatus.near = 0;
    roverStatus.search = 0;
  }
  else if (roverStatus.toGoal == 3) {
    roverSuccess.goalGPS = roverStatus.toGoal;
    roverSuccess.full = 1;
    eeprom.write(eeprom.addrEEPROM, 27, (byte)roverSuccess.goalGPS); //logger//logger

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



//=========LIDAR sensor function============================================================================//
unsigned int getLIDAR() {
  boolean LIDAR_flag = 1;
  int distance;
  int bytenum = 0;
  while (Serial2.available() > 0 && LIDAR_flag == 1)//near_flagは一時的なもの
  {
    byte c = Serial2.read();
    switch (bytenum) {
      case 0://frame header must be 0x59
        //        Serial.print("Byte0:");
        //        Serial.println(c,HEX);
        if (c == 0x59) {
          bytenum += 1;
        }
        break;
      case 1://frame header must be 0x59
        //        Serial.print("Byte1:");
        //        Serial.println(c,HEX);
        if (c == 0x59) {
          bytenum += 1;
        }
        break;
      case 2://distance value low 8 bits
        //        Serial.print("Byte2:");
        //        Serial.println(c,HEX);
        if (c == 0x59) {
          //多分次がcase2
        }
        else {
          distance = (int)c;
          bytenum += 1;
        }
        break;
      case 3://distance value high 8 bits
        //        Serial.print("Byte3:");
        //        Serial.println(c,HEX);
        distance = distance + 256 * (int)c;
        //        Serial.print("distance:");
        //        Serial.println(cm_LIDAR);
        LIDAR_flag = 0;
        bytenum += 1;
        break;
      case 4://strength value low 8 bits
        //        Serial.print("Byte4:");
        //        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 5://strength value high 8 bits
        //        Serial.print("Byte5:");
        //        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 6://Temp_L low 8 bits
        //        Serial.print("Byte6:");
        //        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 7://Temp_H high 8 bits
        //        Serial.print("Byte7:");
        //        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 8://checksum
        //        Serial.print("Byte8:");
        //        Serial.println(c,HEX);
        bytenum = 0;
        break;
    }
  }
  LIDAR_flag = 1;
  if (0 < distance && distance < 1000) {
    LIDAR_buf = distance;
    return distance;
  } else {
    return LIDAR_buf;
  }
}

//=========GPS and position function============================================================================//
float deg2rad(float deg)
{
  return (float)(deg * PI / 180.0);
}


// void updateRange_deg(){
//   degRtoA = atan2((lngR - LngA) * 1.23, (latR - LatA)) * 57.3 + 180;
//   rangeRtoA = gps.distanceBetween(latR,lngR,LatA,LngA);
// }
// Hubenyの式

float calculateDistance(float latitude1, float longitude1, float latitude2, float longitude2)
{
  // 先に計算しておいた定数
  float e2 = 0.00669437999019758;   // WGS84における「離心率e」の2乗
  float Rx = 6378137.0;             // WGS84における「赤道半径Rx」
  float m_numer = 6335439.32729246; // WGS84における「子午線曲率半径M」の分子(Rx(1-e^2))

  float rad_lat1 = deg2rad(latitude1);
  float rad_lon1 = deg2rad(longitude1);
  float rad_lat2 = deg2rad(latitude2);
  float rad_lon2 = deg2rad(longitude2);

  float dp = (float)(rad_lon1 - rad_lon2);        // 2点の緯度差
  float dr = (float)(rad_lat1 - rad_lat2);        // 2点の経度差
  float p = (float)((rad_lon1 + rad_lon2) * 0.5); // 2点の平均緯度

  float w = (float)sqrt(1.0 - e2 * pow(sin(p), 2));
  float m = (float)(m_numer / pow(w, 3)); // 子午線曲率半径
  float n = (float)(Rx / w);              // 卯酉(ぼうゆう)線曲率半径

  // 2点間の距離(単位m)
  float d = (float)sqrt(pow((m * dp), 2) + pow((n * cos(p) * dr), 2));
  return d;
}


float angle_calculation() {
  x = atan2(imu.yMag - imu.caliby, imu.xMag - imu.calibx) / 3.14 * 180 + 180; //磁北を0°(or360°)として出力
  x += imu.calib;
  x -= 7; //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

  // calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整

  if (x > 360)
  {
    x -= 360;
  }

  else if (x < 0)
  {
    x += 360;
  }

  else
  {
    //x = x;
  }

  // バッファに取り込んで、インデックスを更新する。
  buf[index] = x;
  index = (index + 1) % BUF_LEN;
  //フィルタ後の値を計算
  filterVal = medianFilter();
  return filterVal;
}

// Medianフィルタ関数
int medianFilter()
{
  //ソート用のバッファ
  static int sortBuf[BUF_LEN];

  //ソート用バッファにデータをコピー
  for (int i = 0; i < BUF_LEN; i++)
  {
    sortBuf[i] = buf[i];
  }

  //クイックソートで並べ替える
  qsort(sortBuf, BUF_LEN, sizeof(int), quicksortFunc);

  return sortBuf[(int)BUF_LEN / 2];
}

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

//クイックソート関数
int quicksortFunc(const void *a, const void *b)
{
  return *(int *)a - *(int *)b;
}


//=========Motor Control function============================================================================//

void motor_angle_go()
{
  digitalWrite(ENABLE, HIGH); // enable
  digitalWrite(CH2, LOW);
  digitalWrite(CH4, LOW);
  if (x < degRtoA) {
    deltaTheta = degRtoA - x;
    Serial.print(":x < degRtoA:");
    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= threshold / 2) || (360 - threshold / 2 <= deltaTheta && deltaTheta <= degRtoA)) {
      speedR = nominalSpeed;
      speedL = nominalSpeed;
      analogWrite( CH1, speedR );
      analogWrite( CH3, speedL );
      Serial.print(":Go straight");
      controlStatus = 1;//"Go straight"
    }
    //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else if (threshold / 2 < deltaTheta && deltaTheta <= 180) {
      speedR = nominalSpeed - (deltaTheta * nominalSpeed / 180);
      speedL = nominalSpeed;
      analogWrite( CH1, speedR );
      analogWrite( CH3, speedL );
      Serial.print(":turn right");
      controlStatus = 3;//"turn right"
    }

    //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else {
      speedR = nominalSpeed;
      speedL = nominalSpeed - ((360 - deltaTheta) * nominalSpeed / 180);
      analogWrite( CH1, speedR );
      analogWrite( CH3, speedL );
      Serial.print(":turn left");
      controlStatus = 2;//"turn left"
    }
  }

  else {
    deltaTheta = x - degRtoA;
    Serial.print(":degRtoA < x:");
    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= threshold / 2) || (360 - threshold / 2 <= deltaTheta && deltaTheta <= 360)) {
      speedR = nominalSpeed;
      speedL = nominalSpeed;
      analogWrite( CH1, speedR );
      analogWrite( CH3, speedL );
      Serial.print(":Go straight");
      controlStatus = 1;//"Go straight"
    }
    //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else if (threshold / 2 < deltaTheta && deltaTheta <= 180) {
      speedR = nominalSpeed;
      speedL = nominalSpeed - (deltaTheta * nominalSpeed / 180);
      analogWrite( CH1, speedR );
      analogWrite( CH3, speedL );
      Serial.print(":turn left");
      controlStatus = 2;//"turn left"
    }

    //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else {
      speedR = nominalSpeed - ((360 - deltaTheta) * nominalSpeed / 180);
      speedL = nominalSpeed;
      analogWrite( CH1, speedR );
      analogWrite( CH3, speedL );
      Serial.print(":turn right");
      controlStatus = 3;//"turn right"

    }
  }
  Serial.print(":speedL:");
  Serial.print(speedL);
  Serial.print(":speedR:");
  Serial.print(speedR);
}


void motor_angle_spin()
{
  if (x < degRtoA) {
    deltaTheta = degRtoA - x;
    Serial.print(":x < degRtoA:");
    Serial.print(deltaTheta);

    if ((0 <= deltaTheta && deltaTheta <= spinThreshold / 2) || (360 - spinThreshold / 2 <= deltaTheta && deltaTheta <= degRtoA)) {
      //閾値内にあるときは真っ直ぐ
      if ((0 <= deltaTheta && deltaTheta <= threshold / 2) || (360 - threshold / 2 <= deltaTheta && deltaTheta <= degRtoA)) {
        speedR = slowSpeed;
        speedL = slowSpeed;
        //スロー前進
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite( CH1, speedR );
        analogWrite( CH2, 0);
        analogWrite( CH3, speedL );
        analogWrite( CH4, 0);
        Serial.print(":Go straight");
        controlStatus = 1;//"Go straight"
        forwardCount += 1;
      }
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else if (threshold / 2 < deltaTheta && deltaTheta <= 180) {
        speedR = slowSpeed - (deltaTheta * slowSpeed / 180);
        speedL = slowSpeed;
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite( CH1, speedR );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speedL );
        analogWrite( CH4, 0 );
        Serial.print(":turn right");
        controlStatus = 3;//"turn right"
      }

      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else {
        speedR = slowSpeed;
        speedL = slowSpeed - ((360 - deltaTheta) * slowSpeed / 180);
        analogWrite( CH1, speedR );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speedL );
        analogWrite( CH4, 0 );
        Serial.print(":turn left");
        controlStatus = 2;//"turn left"
        spinCount += 1;
      }
    }
    else {
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      if (threshold / 2 < deltaTheta && deltaTheta <= 180) {
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, 0);
        analogWrite(CH2, verySlowSpeed);
        analogWrite(CH3, verySlowSpeed);
        analogWrite(CH4, 0);
        Serial.print(":spin to right");
        controlStatus = 4;//"spin to right"
        spinCount += 1;
      }

      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else {
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, verySlowSpeed);
        analogWrite(CH2, 0);
        analogWrite(CH3, 0);
        analogWrite(CH4, verySlowSpeed);
        Serial.print(":spin to left");
        controlStatus = 5;//"spin to left
        spinCount += 1;
      }
    }
  }

  else {
    deltaTheta = x - degRtoA;
    Serial.print(":degRtoA < x:");
    Serial.print(deltaTheta);
    if ((0 <= deltaTheta && deltaTheta <= spinThreshold / 2) || (360 - spinThreshold / 2 <= deltaTheta && deltaTheta <= 360)) {
      //閾値内にあるときは真っ直ぐ
      if ((0 <= deltaTheta && deltaTheta <= threshold / 2) || (360 - threshold / 2 <= deltaTheta && deltaTheta <= 360)) {
        speedR = slowSpeed;
        speedL = slowSpeed;
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite( CH1, speedR );
        analogWrite( CH2, 0);
        analogWrite( CH3, speedL );
        analogWrite( CH4, 0);
        Serial.print(":Go straight");
        controlStatus = 1;//"Go straight"
        forwardCount += 1;
      }
      //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else if (threshold / 2 < deltaTheta && deltaTheta <= 180) {
        speedR = slowSpeed;
        speedL = slowSpeed - (deltaTheta * slowSpeed / 180);
        analogWrite( CH1, speedR );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speedL );
        analogWrite( CH4, 0 );
        Serial.print(":turn left");
        controlStatus = 2;//"turn left"
      }

      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else {
        speedR = slowSpeed - ((360 - deltaTheta) * slowSpeed / 180);
        speedL = slowSpeed;
        analogWrite( CH1, speedR );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speedL );
        analogWrite( CH4, 0 );
        Serial.print(":turn right");
        controlStatus = 3;//"turn right"
      }
    }
    else {
      if (threshold / 2 < deltaTheta && deltaTheta <= 180) {
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, verySlowSpeed);
        analogWrite(CH2, 0);
        analogWrite(CH3, 0);
        analogWrite(CH4, verySlowSpeed);
        Serial.print(":spin to left");
        controlStatus = 5;//"spin to left
        spinCount += 1;
      }
      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else {
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, 0);
        analogWrite(CH2, verySlowSpeed);
        analogWrite(CH3, verySlowSpeed);
        analogWrite(CH4, 0);
        Serial.print(":spin to right");
        controlStatus = 4;//"spin to right"
        spinCount += 1;
      }
    }
  }
  Serial.print(":speedL:");
  Serial.print(speedL);
  Serial.print(":speedR:");
  Serial.print(speedR);
  Serial.print(":forwardCount:");
  Serial.print(forwardCount);
}

//============SDCard function=========================================================================//
void LogToSDCard() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(overallTime);
    dataFile.print(",");
    dataFile.print(imu.xMag);
    dataFile.print(",");
    dataFile.print(imu.yMag);
    dataFile.print(",");
    dataFile.print(imu.calibx);
    dataFile.print(",");
    dataFile.print(imu.caliby);
    dataFile.print(",");
    dataFile.print(x);
    dataFile.print(",");
    dataFile.print(cm_long);
    dataFile.print(",");
    dataFile.print(latR);
    dataFile.print(",");
    dataFile.print(lngR);
    dataFile.print(",");
    dataFile.print(degRtoA);
    dataFile.print(",");
    dataFile.print(controlStatus);
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

  sendData.writeToTwelite(imu,x,cm_LIDAR,latR,lngR,degRtoA,controlStatus,overallTime);//send HK firstly
  Serial.println("Data transmission");

  commStop = millis();
  while (1) { //then go into waiting loop for ACK or NACK
    commStart = millis();
    if (commStart > commStop + 100) { //if 20ms passes, then send HK again
      sendData.writeToTwelite(imu,x,cm_LIDAR,latR,lngR,degRtoA,controlStatus,overallTime);
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
            sendData.writeToTwelite(imu,x,cm_LIDAR,latR,lngR,degRtoA,controlStatus,overallTime);
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
