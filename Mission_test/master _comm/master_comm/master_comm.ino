//Header file
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

//------------------------------Ultrasonic sensor------------------------------
//Ultrasonic sensor(short)Front
unsigned int cm;
const int HEAD_Trig = 22;
const int HEAD_Echo = 24;

//Ultrasonic sensor(long)Front
#define HEADpin A15
unsigned int anVolt;
unsigned int cm_long;
int emergencyStopDist = 10;

//Buffer for one range measurement near goal
#define MEAS_BUF_LEN  10//it should be more than 10, length of 9axis basic buffer
int bufcm[MEAS_BUF_LEN];
int measureIndex = 0;

//Buffer for all range measurement near goal
#define SEAR_BUF_LEN 20
int listcm[SEAR_BUF_LEN];
int searchIndex = 0;


//Ultrasonic sensor(short)Bottom
const int BOTTOM_Trig = 6;
const int BOTTOM_Echo = 7;

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

//------------------------------9axis sensor------------------------------
// I2C address for BMX055 Gyro
#define Addr_Gyro 0x69 // (JP1,JP2,JP3 = Openの時)
// I2C address for BMX055 Magnetic
#define Addr_Mag 0x13 // (JP1,JP2,JP3 = Openの時)

// センサーの値を保存するグローバル変数
float xGyro = 0.00;
float yGyro = 0.00;
float zGyro = 0.00;
int xMag = 0;
int yMag = 0;
int zMag = 0;

//Constant for calibration(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calib = 175;
double Calibx = 20;
double Caliby = 132;

float x; //ローバーの慣性姿勢角
float deltaTheta;//目的方向と姿勢の相対角度差
int threshold = 10; //角度の差分の閾値
int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)

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


//------------------------------GPS sensor------------------------------
TinyGPSPlus gps;
// double LatA = 35.7100069, LngA = 139.8108103;  //目的地Aの緯度経度(スカイツリー)
//double LatA = 35.7142738, LngA = 139.76185488809645; //目的地Aの緯度経度(2号館)
//double LatA = 35.7140655517578, LngA = 139.7602539062500; //目的地Aの緯度経度(工学部広場)
double LatA = 35.719970703125, LngA = 139.7361145019531; //目的地Aの緯度経度((教育の森公園)
double LatR = 35.715328, LngR = 139.761138;  //現在地の初期想定値(7号館屋上)
float degRtoA; //GPS現在地における目的地の慣性方角
float rangeRtoA;


//------------------------------EEPROM------------------------------
//デバイスアドレス(スレーブ)
uint8_t addrEEPROM = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
unsigned int addrData = 30; //書き込むレジスタ(0x0000~0xFFFF全部使える) (0~30は目的地のGPSデータとステータスを保管する)

//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;


//------------------------------Onboard Camera------------------------------



//------------------------------Communication--------------------------------------------
// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400

//Define Structures for receiving and Handling Rover data
typedef struct _roverData {
  uint8_t roverComsStat;
  int xMag;
  int yMag;
  uint16_t calibX;
  uint16_t calibY;
  float x;
  uint16_t cmLong;
  float latR;
  float lngR;
  float degRtoA;
  byte statusControl;
  unsigned long int time;
} roverData;

typedef union _packetData {
  roverData message;
  unsigned char packetData[sizeof(roverData)];
} packetData;

typedef struct _gpsDataStruct {
  float latA[3];
  float lngA[3];
} gpsDataStruct;

typedef union _gpsPacketUnion {
  gpsDataStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsDataStruct)];
} gpsPacketUnion;

const int MaxBufferSize = 160;
char buffRx[MaxBufferSize];
void Parse();
int bufferPos = 0;
packetData packetTx;//Packet to be coded and then written to twelite
gpsPacketUnion dataRx;//Received packet with GPS data

void  writeToTwelite();
void encodeCyclic();
uint8_t encodedTx[2 * sizeof(roverData)]; //Encoded message to be sent
uint8_t encodedRx[2 * sizeof(gpsDataStruct)];
const uint8_t generator[4] = {0x46, 0x23, 0x17, 0x0D};
const uint8_t parityCheck[3] = {0x5C, 0x72, 0x39};
unsigned long start;
unsigned long stopi;
void readRoverData();
char encodedReceived[2 * sizeof(roverData)];

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
  BMX055_Init();

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
  degRtoA = atan2((LngR - LngA) * 1.23, (LatR - LatA)) * 57.3 + 180;
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
    if (Serial2.available() > 0) {
      char c = Serial2.read();
      //Serial2.print(c);
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) { //read as data in one packet before it receives "\n"
        buffRx[bufferPos] = c;
        bufferPos++;
        buffRx[bufferPos] = '\0';
      }
      else //Check the buffa if it reads the last character in one packet
      {
        if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '0') { //Arbitrary packet for Rover
          if (buffRx[6] == '2') { //NACK
            //do nothing
          }
          else if (buffRx[6] == '1') { //ACK
            //do nothing
          }
          else if (buffRx[6] == '0') { //DATARECEIVE
            processData();//character data is converted to uint8_t data here and is stored in the encodedRx[] buffer
            decodeCyclic();//decode GPS data of three goals


            Serial.println("------------------------initial MODE SUCCESS!!!------------------------");
            Serial.print("1st GPS:");
            Serial.print(dataRx.gpsData.latA[0]);
            Serial.print(",");
            Serial.println(dataRx.gpsData.lngA[0]);
            Serial.print("2nd GPS:");
            Serial.print(dataRx.gpsData.latA[1]);
            Serial.print(",");
            Serial.println(dataRx.gpsData.lngA[1]);
            Serial.print("3rd GPS:");
            Serial.print(dataRx.gpsData.latA[2]);
            Serial.print(",");
            Serial.println(dataRx.gpsData.lngA[2]);
            Serial.println("---------------------------------------------------------------------");


            LogGPSdata();//log the gps data of destination to EEPROM
            Serial2.print(":000101X\r\n"); //Send ACK to MC
            goalCalculation();//calculate distance to goals and decide root

            int first = goalRoute[0];//set first goal to the destination
            LatA = dataRx.gpsData.latA[first];
            LngA = dataRx.gpsData.lngA[first];

            roverStatus.initial = 0;
            roverStatus.toGoal = 1;
          }
        }
        //Serial.println(buffRx);
        bufferPos = 0;
      }
    }
  }

  //=================================Nominal Mode=================================


  //---------------------9軸取得--------------------------------------------------
  // BMX055 ジャイロの読み取り
  BMX055_Gyro();
  // BMX055 磁気の読み取り
  BMX055_Mag();

  //キャリブレーションが終了しているなら
  if (roverStatus.calibration == 0) {
    x = angle_calculation();
  }

  //---------------------LIDARセンサ取得--------------------------------------------------
  cm_LIDAR = getLIDAR();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  cm = getUltrasonic_HEAD();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  anVolt = analogRead(HEADpin);
  cm_long = anVolt / 2;

  //---------------------GPS acquisition--------------------------------------------------
  updateGPSlocation();
  degRtoA = atan2((LngR - LngA) * 1.23, (LatR - LatA)) * 57.3 + 180;
  rangeRtoA = gps.distanceBetween(LatR, LngR, LatA, LngA);

  //---------------------Check parameter & update Status--------------------------------------------------

  Serial.print(":LatR:");
  Serial.print(LatR);
  Serial.print(":LngR:");
  Serial.print(LngR);
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
  if (cm < emergencyStopDist) {
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
    Serial.print(xMag);
    Serial.print(":yMag:");
    Serial.print(yMag);
    Serial.print(":calIndex:");
    Serial.print(calIndex);
    bufx[calIndex] = xMag;
    bufy[calIndex] = yMag;
    if (calIndex == CAL_BUF_LEN - 1) { //バッファに値がたまったら
      Calibx = xcenter_calculation();
      Caliby = ycenter_calculation();
      roverStatus.calibration = 0 ;
      Serial.print(":Calib_x:");
      Serial.print(Calibx);
      Serial.print(":Calib_y:");
      Serial.print(Caliby);
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
    LogToEEPROM();
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
    range[i] = gps.distanceBetween(LatR, LngR, dataRx.gpsData.latA[i], dataRx.gpsData.lngA[i]);
    goalRoute[i] = i + 1;
  }
  sortRange(range, goalRoute); //root = [1,2,3]
  writeEEPROM(addrEEPROM, 24, (byte)goalRoute[0]);
  writeEEPROM(addrEEPROM, 25, (byte)goalRoute[1]);
  writeEEPROM(addrEEPROM, 26, (byte)goalRoute[2]);
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
    writeEEPROM(addrEEPROM, 27, (byte)roverSuccess.goalGPS); //logger

    int next = goalRoute[roverStatus.toGoal];//set next goal to the destination
    roverStatus.toGoal += 1;
    LatA = dataRx.gpsData.latA[next];
    LngA = dataRx.gpsData.lngA[next];

    roverStatus.near = 0;
    roverStatus.search = 0;
  }
  else if (roverStatus.toGoal == 3) {
    roverSuccess.goalGPS = roverStatus.toGoal;
    roverSuccess.full = 1;
    writeEEPROM(addrEEPROM, 27, (byte)roverSuccess.goalGPS); //logger//logger

    roverStatus.near = 0;
    roverStatus.search = 0;
    roverMode.sleep = 1;
  }
  return;
}

//=========Ultrasonic sensor function============================================================================//
unsigned int getUltrasonic_HEAD() {
  long duration;
  digitalWrite(HEAD_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(HEAD_Trig, HIGH);
  delayMicroseconds(10);
  duration = pulseIn(HEAD_Echo, HIGH);
  return microsecTocm(duration);
}

unsigned int microsecTocm(long microsec) {
  return (unsigned int) microsec / 29 / 2;
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

double deg2rad(double deg)
{
  return (double)(deg * PI / 180.0);
}

void updateGPSlocation() {
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
      LatR = gps.location.lat();  // roverの緯度を計算
      LngR = gps.location.lng(); // roverの経度を計算
      break;
    }
    //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
    delay(1);
  }
}

// void updateRange_deg(){
//   degRtoA = atan2((LngR - LngA) * 1.23, (LatR - LatA)) * 57.3 + 180;
//   rangeRtoA = gps.distanceBetween(LatR,LngR,LatA,LngA);
// }
// Hubenyの式

float calculateDistance(double latitude1, double longitude1, double latitude2, double longitude2)
{
  // 先に計算しておいた定数
  double e2 = 0.00669437999019758;   // WGS84における「離心率e」の2乗
  double Rx = 6378137.0;             // WGS84における「赤道半径Rx」
  double m_numer = 6335439.32729246; // WGS84における「子午線曲率半径M」の分子(Rx(1-e^2))

  double rad_lat1 = deg2rad(latitude1);
  double rad_lon1 = deg2rad(longitude1);
  double rad_lat2 = deg2rad(latitude2);
  double rad_lon2 = deg2rad(longitude2);

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


//===========9axis sensor function==========================================================================//
void BMX055_Init()
{
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F); // Select Range register

  //ここでWire.write()の中の値を変えることでスケール変更可能
  // 0x00, 2000deg/s
  // 0x01, 1000deg/s
  // 0x02, 500deg/s
  // 0x03, 250deg/s
  // 0x04, 125deg/s

  Wire.write(0x04); // Full scale = +/- 500 degree/s

  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10); // Select Bandwidth register
  Wire.write(0x07); // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11); // Select LPM1 register
  Wire.write(0x00); // Normal mode, sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x83); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B); // Select Mag register
  Wire.write(0x01); // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C); // Select Mag register
  Wire.write(0x00); // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E); // Select Mag register
  Wire.write(0x84); // X, Y, Z-Axis enabled
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51); // Select Mag register
  Wire.write(0x04); // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52); // Select Mag register
  Wire.write(0x16); // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}


void BMX055_Gyro()
{
  unsigned int data[6];
  for (int i = 0; i < 6; i++)
  {
    Wire.beginTransmission(Addr_Gyro);
    Wire.write((2 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Gyro, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xGyro = (data[1] * 256) + data[0];
  if (xGyro > 32767)
    xGyro -= 65536;
  yGyro = (data[3] * 256) + data[2];
  if (yGyro > 32767)
    yGyro -= 65536;
  zGyro = (data[5] * 256) + data[4];
  if (zGyro > 32767)
    zGyro -= 65536;

  xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
  yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
  zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
}


void BMX055_Mag()
{
  unsigned int data[8];
  for (int i = 0; i < 8; i++)
  {
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i)); // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1); // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
  // Convert the data
  xMag = ((data[1] << 5) | (data[0] >> 3));
  if (xMag > 4095)
    xMag -= 8192;
  yMag = ((data[3] << 5) | (data[2] >> 3));
  if (yMag > 4095)
    yMag -= 8192;
  zMag = ((data[5] << 7) | (data[4] >> 1));
  if (zMag > 16383)
    zMag -= 32768;
}

float angle_calculation() {
  x = atan2(yMag - Caliby, xMag - Calibx) / 3.14 * 180 + 180; //磁北を0°(or360°)として出力
  x += Calib;
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




//============EEPROM function=========================================================================//
void writeEEPROM(int addr_device, unsigned int addr_res, byte data )
{
  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.write(data);
  Wire.endTransmission();
  delay(5);//この遅延はどうやら必要っぽい
}

void EEPROM_write_int(int addr_device, unsigned int addr_res, int data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EEPROM_write_long(int addr_device, unsigned int addr_res, unsigned long data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}


void EEPROM_write_float(int addr_device, unsigned int addr_res, double data) {
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    //    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res + i, p[i]);
  }
  //  Serial.println("");
}

byte readEEPROM(int addr_device, unsigned int addr_res )
{
  byte rdata = 0xFF;

  Wire.beginTransmission(addr_device);
  Wire.write((int)(addr_res >> 8));   // MSB
  Wire.write((int)(addr_res & 0xFF)); // LSB
  Wire.endTransmission();

  Wire.requestFrom(addr_device, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


double EEPROM_read_float(int addr_device, unsigned int addr_res) {
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++) {
    //    Serial.print(i+1);
    //    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res + i);
    //    Serial.println(p_read[i]);
  }
  //  Serial.println("");
  double *d = (double *)p_read;
  double data = *d;
  return data;
}

void LogToEEPROM() {
  EEPROM_write_int(addrEEPROM, addrData, xMag);
  addrData += 2;
  EEPROM_write_int(addrEEPROM, addrData, yMag);
  addrData += 2;
  EEPROM_write_int(addrEEPROM, addrData, Calibx);
  addrData += 2;
  EEPROM_write_int(addrEEPROM, addrData, Caliby);
  addrData += 2;
  EEPROM_write_float(addrEEPROM, addrData, x);
  addrData += 4;
  EEPROM_write_int(addrEEPROM, addrData, cm_long);
  addrData += 2;
  EEPROM_write_float(addrEEPROM, addrData, LatR);
  addrData += 4;
  EEPROM_write_float(addrEEPROM, addrData, LngR);
  addrData += 4;
  EEPROM_write_float(addrEEPROM, addrData, degRtoA);
  addrData += 4;
  writeEEPROM(addrEEPROM, addrData, (byte)controlStatus);
  addrData += 2;
  overallTime = millis();
  EEPROM_write_long(addrEEPROM, addrData, overallTime);
  addrData += 4;
}

void LogGPSdata() {
  EEPROM_write_float(addrEEPROM, 0, dataRx.gpsData.latA[0]);
  EEPROM_write_float(addrEEPROM, 4, dataRx.gpsData.lngA[0]);
  EEPROM_write_float(addrEEPROM, 8, dataRx.gpsData.latA[1]);
  EEPROM_write_float(addrEEPROM, 12, dataRx.gpsData.lngA[1]);
  EEPROM_write_float(addrEEPROM, 16, dataRx.gpsData.latA[2]);
  EEPROM_write_float(addrEEPROM, 20, dataRx.gpsData.lngA[2]);
}

//============SDCard function=========================================================================//
void LogToSDCard() {
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(overallTime);
    dataFile.print(",");
    dataFile.print(xMag);
    dataFile.print(",");
    dataFile.print(yMag);
    dataFile.print(",");
    dataFile.print(Calibx);
    dataFile.print(",");
    dataFile.print(Caliby);
    dataFile.print(",");
    dataFile.print(x);
    dataFile.print(",");
    dataFile.print(cm_long);
    dataFile.print(",");
    dataFile.print(LatR);
    dataFile.print(",");
    dataFile.print(LngR);
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
//=========Communication function============================================================================//
void processData() {
  // character data is converted to uint8_t data here
  // and is stored in the encodedRx[] buffer
  int i = 7;
  int d, e;
  int checker;
  while (i < bufferPos - 4) {//What is lentCtr???
    checker = buffRx[i] & 0b01000000;
    //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
    if (checker == 0b01000000) {
      d = buffRx[i] + 9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
    }
    else {
      d = buffRx[i];
    }
    d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
    d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
    //Same for the following character
    checker = buffRx[i + 1] & 0b01000000;
    if (checker == 0b01000000) {
      e = buffRx[i + 1] + 9;
    }
    else {
      e = buffRx[i + 1];
    }
    e = e & 0b00001111;
    encodedRx[(i - 7) / 2] = d + e;
    i = i + 2;
  }
}

void  writeToTwelite () {
  int ctr1 = 0;
  readRoverData();
  encodeCyclic();
  Serial2.print(":000100");
  //Serial.print(":000100");
  while (ctr1 < 2 * sizeof(roverData)) {
    if ((uint8_t)encodedTx[ctr1] < 16) {
      Serial2.print("0");
      //Serial.print("0");
    }
    Serial2.print(encodedTx[ctr1], HEX);
    //Serial.print(encodedTx[ctr1],HEX);
    ctr1++;
  }
  Serial2.print("X\r\n");
  //Serial.print("X\r\n");
}

void readRoverData() {
  packetTx.message.roverComsStat = 4; // need to implement
  Serial.print("roverComsStat:");
  Serial.println(packetTx.message.roverComsStat);
  packetTx.message.xMag = xMag;
  Serial.print("xMag:");
  Serial.println(packetTx.message.xMag);
  packetTx.message.yMag = yMag;
  Serial.print("yMag:");
  Serial.println(packetTx.message.yMag);
  packetTx.message.calibX = Calibx;
  Serial.print("Calibx:");
  Serial.println(packetTx.message.calibX);
  packetTx.message.calibY = Caliby;
  Serial.print("Caliby:");
  Serial.println(packetTx.message.calibY);
  packetTx.message.x = x;
  Serial.print("x:");
  Serial.println(packetTx.message.x);
  packetTx.message.cmLong = cm_LIDAR;
  Serial.print("cm_long:");
  Serial.println(packetTx.message.cmLong);
  packetTx.message.latR = LatR;
  Serial.print("LatR:");
  Serial.println(packetTx.message.latR);
  packetTx.message.lngR = LngR;
  Serial.print("LngR:");
  Serial.println(packetTx.message.lngR);
  packetTx.message.degRtoA = degRtoA;
  Serial.print("degRtoA:");
  Serial.println(packetTx.message.degRtoA);
  packetTx.message.statusControl = controlStatus;
  Serial.print("controlStatus:");
  Serial.println(packetTx.message.statusControl);
  packetTx.message.time = overallTime;
  Serial.print("time:");
  Serial.println(packetTx.message.time);
}

void encodeCyclic() {
  uint8_t ctr = 0;
  uint8_t m;
  while (ctr < sizeof(roverData)) {
    m = packetTx.packetData[ctr] >> 4;
    encodedTx[2 * ctr] = ((m & 1) * generator[3]) ^ (((m >> 1) & 1) * generator[2]) ^
                         (((m >> 2) & 1) * generator[1]) ^ (((m >> 3) & 1) * generator[0]);
    //Serial.print(encodedTx[2*ctr],HEX);
    m = packetTx.packetData[ctr];
    encodedTx[2 * ctr + 1] = ((m & 1) * generator[3]) ^ (((m >> 1) & 1) * generator[2]) ^
                             (((m >> 2) & 1) * generator[1]) ^ (((m >> 3) & 1) * generator[0]);
    //Serial.println(encodedTx[2*ctr+1],HEX);
    ctr++;
  }
}

bool checkError(uint8_t dataByte) {
  uint8_t p[3];
  uint8_t ctr = 0;
  p[0] = dataByte & parityCheck[0];
  p[1] = dataByte & parityCheck[1];
  p[2] = dataByte & parityCheck[2];
  while (ctr < sizeof(gpsDataStruct) / 2) {
    p[0] = (p[0] & 1) ^ (p[0] >> 1);
    p[1] = (p[1] & 1) ^ (p[1] >> 1);
    p[2] = (p[2] & 1) ^ (p[2] >> 1);
    ctr++;
  }
  return (p[0] > 0) || (p[1] > 0) || (p[2] > 0);
}

boolean decodeCyclic() {
  uint8_t ctr = 0;
  bool error[2];
  while (ctr < sizeof(gpsDataStruct)) {
    encodedRx[2 * ctr] = encodedRx[2 * ctr] & 0x7F;
    encodedRx[2 * ctr + 1] = encodedRx[2 * ctr + 1] & 0x7F;
    error[0] = checkError(encodedRx[2 * ctr]);
    error[1] = checkError(encodedRx[2 * ctr + 1]);
    dataRx.gpsBytes[ctr] = ((encodedRx[2 * ctr] << 1) & 0xF0) +
                           ((encodedRx[2 * ctr + 1] >> 3) & 0x0F); //populate GPS data of goals in dataRx
    if (error[0] || error[1]) { //NACK
      Serial2.print(":000102X\r\n");
      return true;
    }
    ctr++;
  }
  //If no errors send ACK
  Serial2.print(":000101X\r\n");
  return false;
}

void commToGS() {
  unsigned long commStart;
  unsigned long commStop;

  writeToTwelite();//send HK firstly
  Serial.println("Data transmission");

  commStop = millis();
  while (1) { //then go into waiting loop for ACK or NACK
    commStart = millis();
    if (commStart > commStop + 100) { //if 20ms passes, then send HK again
      writeToTwelite();
      Serial.println("timeout:100ms");
      break;
    }
    if (Serial2.available() > 0) {
      char c = Serial2.read();
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) {
        buffRx[bufferPos] = c;
        bufferPos++;
      }
      else
      {
        buffRx[bufferPos] = '\0';
        //Checks
        if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '0') { //Arbitrary packet for Rover
          //Serial.println(Buffer);
          if (buffRx[6] == '2') { //NACK
            Serial.print("NACK: Resending packet...");
            writeToTwelite();
          } else if (buffRx[6] == '1') { //ACK
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
