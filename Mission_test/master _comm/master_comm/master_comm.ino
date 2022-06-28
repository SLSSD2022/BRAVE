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
int emergency_stop_distance = 10;

//Buffer for one range measurement near goal
#define MEAS_BUF_LEN  10//it should be more than 10, length of 9axis basic buffer
int bufcm[MEAS_BUF_LEN];
int meas_index = 0;

//Buffer for all range measurement near goal
#define SEAR_BUF_LEN 20
int listcm[SEAR_BUF_LEN];
int search_index = 0;


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

int Normal_speed = 250;
int Slow_speed = 200;
int Very_Slow_speed = 150;
int speed_R;
int speed_L;
int count_forward = 0;

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

//Constant for Calibration(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
double Calib = 175; 
double Calibx = 20;
double Caliby = 132;

float x; //ローバーの慣性姿勢角
float delta_theta;//目的方向と姿勢の相対角度差
int threshold = 10; //角度の差分の閾値
int spin_threshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)

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
int cal_index = 0;
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
uint8_t Addr_eeprom = 0x50;//24lC1025の場合1010000(前半)or1010100(後半)を選べる
unsigned int DATA_ADDRESS = 30; //書き込むレジスタ(0x0000~0xFFFF全部使える) (0~30は目的地のGPSデータとステータスを保管する)

//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;


//------------------------------Onboard Camera------------------------------



//------------------------------Communication--------------------------------------------
// Set pins for reset and Baud rate speed of Twelite
int RST = 2;
int BPS = 3; // if HIGH set Baud rate to 115200 at MWSerial, if LOW to 38400

//Define Structures for receiving and Handling Rover data
typedef struct roverData{
  uint8_t roverComsStat;
  uint16_t xMag;
  uint16_t yMag;
  uint16_t calibX;
  uint16_t calibY;
  float x;
  uint16_t cmLong;
  float latR;
  float lngR;
  float degRtoA;
  byte statusControl;
  unsigned long int time;
};
  
typedef union packetData{
  roverData message;
  unsigned char packetData[sizeof(roverData)];
};

typedef struct gpsDataStruct{
  float latA[3];
  float lngA[3];
};

typedef union gpsPacketUnion{
  gpsDataStruct gpsData;
  uint8_t gpsBytes[sizeof(gpsDataStruct)];
};

const int MaxBufferSize = 160;
char buffRx[MaxBufferSize];
void Parse();
int bufferPos = 0;
packetData packetTx;//Packet to be coded and then written to twelite 
gpsPacketUnion dataRx;//Received packet with GPS data

void  writeToTwelite();
void encodeCyclic();
uint8_t encodedTx[2*sizeof(roverData)];//Encoded message to be sent 
uint8_t encodedRx[2*sizeof(gpsDataStruct)];
const uint8_t generator[4] = {0x46,0x23,0x17,0x0D};
const uint8_t parityCheck[3] = {0x5C,0x72,0x39};
int commsStart;
int commsStop;
unsigned long start;
unsigned long stopi;
void readRoverData();
void  writeToTwelite();
char encodedReceived[2*sizeof(roverData)];

///-----------------------------Control Status, HK-----------------------------
typedef struct modeStruct{
  unsigned char Manual : 1;
  unsigned char Auto_GPSonly : 1;
  unsigned char Auto_Aggressive : 1;
  unsigned char Sleep : 1;
};


//Status using bit field
typedef struct statusStruct{
    unsigned char Initial : 1;
    unsigned char Calibration : 1;
    unsigned int toGoal;
    unsigned char Near : 1;
    unsigned char Search : 1;
    unsigned char Sleep : 1;
};


typedef struct successStruct{
    unsigned char GPSreceive : 1;
    unsigned int Goal_GPS;
    unsigned int Goal_0m;
    unsigned char full : 1;
} ;

modeStruct roverMode = {0,1,0,0};
statusStruct roverStatus = {1,1,0,0,0,0};
successStruct roverSuccess = {0,0,0,0};

boolean Communication_flag = 1;
boolean Stop_flag = 0;

int count_search = 0;//探索中のシーケンス管理カウント
const int spin_iteration = 1;//探索中のスピン移動に使うループ回数
const int spinmove_iteration = 1;//探索後のスピン移動に使うループ回数

int count_spin = 0;//探索後、方向に向けてスピン回数のカウント
int wait_spin = 0;//探索後、スピン後停止する回数のカウント
const int wait_iteration = 10;//探索後、スピン後少し停止するループ数(10よりは大きくする)
const int forward_iteration = 20;//探索後、方向に向かって進むループ数

int goal_rout[3];

int Memory_flag = 0;
int Status_control;
unsigned long time;


/*
############################################################################################################
############################################################################################################
*/

void setup()
{
    // デバッグ用シリアル通信は9600bps
  Serial.begin(115200);//ステータス設定(試験したい状況)
  Communication_flag = 1;
  roverStatus.Calibration = 1;
  roverStatus.Near = 0;//ゴール5m付近のとき
  roverStatus.Search = 0;//ゴール5m付近で測距するとき
//  while(1){
//    anVolt = analogRead(HEADpin);
//    cm_long = anVolt/2;
//    Serial.println(cm_long);
//    delay(1000);
//  }
  
  //超音波センサ
  pinMode(HEAD_Trig,OUTPUT);
  pinMode(HEAD_Echo,INPUT);
  
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
  
  //SDcard initialization
  pinMode(SDSW,INPUT_PULLUP);
  while(1){
    if(digitalRead(SDSW) == 0){
      Serial.println("Card inserted!");
      break;
    }
    else{
      Serial.println("Card not inserted!");
    }
    delay(1000);
  }
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");

  
  //初期値
  degRtoA = atan2((LngR - LngA) * 1.23, (LatR - LatA)) * 57.3 + 180;
  //ログを初期化(この方法だとめっちゃ時間かかるので今後改善が必要)
//  unsigned long k = 0; 
//  while(k < 6000){
//    writeEEPROM(Addr_eeprom, DATA_ADDRESS,0);
//    DATA_ADDRESS += 1;
//    k +=1;
//    Serial.println(k);
//  }
//  DATA_ADDRESS = 0;
  //バッファの初期化
  for(int i=0; i<CAL_BUF_LEN; i++) {
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
  //=================================Initial Mode=================================
  while (roverStatus.Initial){
    start = millis();
    if (start> stopi + 1000){
      Serial.println("Initial Mode");
      stopi = millis();
    }
    if (Serial2.available() > 0){
      char c = Serial2.read();
      //Serial2.print(c);
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ){//read as data in one packet before it receives "\n"
        buffRx[bufferPos] = c;
        bufferPos++;
        buffRx[bufferPos] = '\0';
      }
      else //Check the buffa if it reads the last character in one packet
      {
        if (buffRx[3]=='0' && buffRx[4]=='1' && buffRx[5]=='0'){    //Arbitrary packet for Rover 
          if (buffRx[6]=='2'){//NACK
            //do nothing
          }
          else if (buffRx[6]=='1'){//ACK
            //do nothing
          }
          else if (buffRx[6]=='0'){//DATARECEIVE
            processData();//character data is converted to uint8_t data here and is stored in the encodedRx[] buffer
            decodeCyclic();//decode GPS data of three goals


            Serial.println("------------------------INITIAL MODE SUCCESS!!!------------------------");
            Serial.println(dataRx.gpsData.latA[0]);//decide which goal to go first
            Serial.println(dataRx.gpsData.latA[1]);//decide which goal to go first
            Serial.println(dataRx.gpsData.latA[2]);//decide which goal to go first
            Serial.println("---------------------------------------------------------------------");


            LogGPSdata();//log the gps data of destination to EEPROM
            
            goal_calculation();//calculate distance to goals and decide root

            int first = goal_rout[0];//set first goal to the destination
            LatA = dataRx.gpsData.latA[first];
            LngA = dataRx.gpsData.lngA[first];

            roverStatus.Initial = 0;
            roverStatus.toGoal = 1;
          }
        }
        Serial.println(buffRx);
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
  if(roverStatus.Calibration == 0){
    x = angle_calculation(); 
  }

  //---------------------LIDARセンサ取得--------------------------------------------------
  cm_LIDAR = getLIDAR();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  cm = getUltrasonic_HEAD();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  anVolt = analogRead(HEADpin);
  cm_long = anVolt/2;
  
  //---------------------GPS acquisition--------------------------------------------------
  updateGPSlocation();
  degRtoA = atan2((LngR - LngA) * 1.23, (LatR - LatA)) * 57.3 + 180;
  rangeRtoA = gps.distanceBetween(LatR,LngR,LatA,LngA);

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
  if(rangeRtoA < 1.0){
    if(roverMode.Auto_GPSonly){
      success_management();
    }
    else if(roverMode.Auto_Aggressive){
      roverStatus.Near = 1;
      roverStatus.Search = 1;
    }                                                                                                                               
  }
  Serial.print(":cm_LIDAR:");
  Serial.print(cm_LIDAR);
  if(cm< emergency_stop_distance){
    Stop_flag = 1;                                                                                                                               
  }else{
    Stop_flag = 0;
  }
  if(roverMode.Sleep == 1){
    Stop_flag = 1;
  }

  //---------------------Special control for each status------------------------------------------------------
  
  
  //Calibration
  if(roverStatus.Calibration == 1){
    Serial.print(":xMag:");
    Serial.print(xMag);
    Serial.print(":yMag:");
    Serial.print(yMag);
    Serial.print(":cal_index:");
    Serial.print(cal_index);
    bufx[cal_index] = xMag;
    bufy[cal_index] = yMag;
    if(cal_index == CAL_BUF_LEN-1){//バッファに値がたまったら
      Calibx = xcenter_calculation();
      Caliby = ycenter_calculation();
      roverStatus.Calibration = 0 ;
      Serial.print(":Calib_x:");
      Serial.print(Calibx);
      Serial.print(":Calib_y:");
      Serial.print(Caliby);
      x = angle_calculation();//このループ後半のためだけ
    }
    else{
      cal_index = (cal_index+1)%CAL_BUF_LEN;
    }
  }

  //ゴール探索時
  if(roverStatus.Calibration == 0 && roverStatus.Near == 1){
    if(roverStatus.Search == 1){//ゴールの方向がまだ分かってない
      if(count_search < spin_iteration){//スピン段階
        Stop_flag = 0;
        count_search += 1;
      }
      else{
        Stop_flag = 1;//測距中は停止する
        bufcm[meas_index] = cm_LIDAR;
        meas_index = (meas_index+1)%MEAS_BUF_LEN;
        //バッファに値がたまったら
        if(meas_index == 0){
          //filter_angle_search();//フィルタリングした測距値をリストに一組追加する。
          listcm[search_index] = cm_LIDAR;//一番最後の角度がもっともらしい。
          listdeg[search_index] = x;//一番最後の角度がもっともらしい。
          Serial.print(":measure deg:");
          Serial.print(listdeg[search_index]);
          //バッファ番号初期化(中身は放置)
          meas_index = 0;
          count_search = 0;
          search_index = (search_index+1)%SEAR_BUF_LEN;
          //測距リストに値がたまったら
          if(search_index == 0){
            int list_index = goal_angle_search();//リストから測距値の最小値と対応するリスト番号を探す。
            degRtoA = listdeg[list_index];//目的地の方向を決定
            Serial.print(":searching_completed!");
            //リスト番号初期化(中身は放置)
            search_index = 0;
            roverStatus.Search = 0;//探索終了
            Stop_flag = 0;//モーター解放
          }
        }
      }
    }
    else if(roverStatus.Search == 0){//ゴール探索時(ゴールの方向が分かって動いている時)
      if(count_spin < spin_iteration){//設定回数まで連続スピンできる
        Stop_flag = 0;
      }
      else{//設定回数までスピンしたら少し停止する
        Stop_flag = 1;
        wait_spin += 1;
        if(wait_spin > wait_iteration){
          wait_spin = 0;
          count_spin = 0;
          Stop_flag = 0;
        }   
      }
    }
  }



  //---------------------Motor Control--------------------------------------------------
  if(Stop_flag == 1){
    //ブレーキ
    digitalWrite(ENABLE, HIGH);
    digitalWrite(CH1,HIGH);
    digitalWrite(CH2,HIGH);
    digitalWrite(CH3,HIGH);
    digitalWrite(CH4,HIGH);
    Status_control = 0;//"stop"
    Serial.print(":stop!");
  }
  else if(Stop_flag == 0 && roverStatus.Calibration == 1){//キャリブレーション時
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, Slow_speed);
    analogWrite(CH2, 0);
    analogWrite(CH3, 0);
    analogWrite(CH4, Slow_speed);
    Status_control = 7;//"Calibration..."
    Serial.print(":calibration");
  }
  else if(Stop_flag == 0 && roverStatus.Calibration == 0 && roverStatus.Near == 1){//ゴール5m付近時
    if(roverStatus.Search == 1){//スピンしながらコーンを探索
      digitalWrite(ENABLE, HIGH); // enable
      analogWrite(CH1, 0);
      analogWrite(CH2, Very_Slow_speed);
      analogWrite(CH3, Very_Slow_speed);
      analogWrite(CH4, 0);
      Status_control = 8;//"Searching..."
      Serial.print(":searching");
    }
    else{//コーンの方を向く
       if(count_forward > forward_iteration){
        roverStatus.Search = 1;
        count_forward = 0;
        Serial.print(":restart searching");
      }
      else{
        motor_angle_spin();
      }
    }
  }
  else if(Stop_flag == 0 && roverStatus.Calibration == 0 && roverStatus.Near == 0 ){//通常走行時
    motor_angle_go();
  }


  //---------------------Logger------------------------------------------------------
  LogToSDCard();
  if(Memory_flag > 5){
    LogToEEPROM();
    Memory_flag = 0;
  }
  else{
    Memory_flag += 1;
  }

  //---------------------Communication(sending HK for every 10 seconds)------------------------------------------------------
  start = millis();
  
  Serial.print(":start:");
  Serial.print(start);
  Serial.print(":stopi:");
  Serial.print(stopi);
  int timer = start - stopi;
  Serial.print(":time:");
  Serial.println(timer);
  if ( timer > 10000){
    Serial.println("HELLO!!!!!!!!!!!");
    Serial.println(":10s Communication");
    writeToTwelite();//send HK firstly
    Serial.println("writeToTwelite 1st");
    commsStop = millis();
    while(Communication_flag == 1){//then go into waiting loop for ACK or NACK
      commsStart = millis();
      if (commsStart > commsStop + 20){//if 20ms passes, then send HK again
        writeToTwelite();
        Serial.println("writeToTwelite 20ms");
        commsStop = millis();
        Communication_flag = 0;
        //break;
      }
      if (Serial2.available() > 0){
        char c = Serial2.read();
        if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ){
          buffRx[bufferPos] = c;
          bufferPos++;
        }
        else
        {
          buffRx[bufferPos] = '\0';
          //Checks
          if (buffRx[3]=='0' && buffRx[4]=='1' && buffRx[5]=='0'){    //Arbitrary packet for Rover 
            //Serial.println(Buffer);
            if (buffRx[6]=='2'){//NACK
              Serial.print("NACK: Resending packet...");
              writeToTwelite();
              Serial.println("writeToTwelite:NACK");
            } else if (buffRx[6]=='1'){//ACK
              Serial.print("ACKNOWLEDGEMENT!");
              Communication_flag = 0;
              //break;
            }
          } 
          Serial.println(buffRx);
          bufferPos = 0;
        } 
      }
    }
    bufferPos = 0;
    Communication_flag = 1;
    stopi = millis();
    Serial.println(":Communication end!!:");
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

void goal_calculation(){
  //基本方針:最初の時点でどう巡るかを決定する。
  unsigned int range[3];
  updateGPSlocation();
  for(int i =0; i<3 ;i++){
    range[i] = gps.distanceBetween(LatR,LngR,dataRx.gpsData.latA[i],dataRx.gpsData.lngA[i]);
    goal_rout[i] = i + 1;
  }
  sortrange(range,goal_rout);//root = [1,2,3]
  writeEEPROM(Addr_eeprom,24,(byte)goal_rout[0]);
  writeEEPROM(Addr_eeprom,25,(byte)goal_rout[1]);
  writeEEPROM(Addr_eeprom,26,(byte)goal_rout[2]);
  return;
}

void swap(int* a, int* b) {
	int temp;
	temp = *a;
	*a = *b;
	*b = temp;
	return;
}

void sortrange(int* data, int* array) {
	if (data[0] < data[1]) swap(&array[0], &array[1]);
	if (data[0] < data[2]) swap(&array[0], &array[2]);
	if (data[1] < data[2]) swap(&array[1], &array[2]);
	return;
}

void success_management(){
  if(roverStatus.toGoal < 3){
    roverSuccess.Goal_GPS = roverStatus.toGoal;
    writeEEPROM(Addr_eeprom,27,(byte)roverSuccess.Goal_GPS);//logger

    int next = goal_rout[roverStatus.toGoal];//set next goal to the destination
    roverStatus.toGoal += 1;
    LatA = dataRx.gpsData.latA[next];
    LngA = dataRx.gpsData.lngA[next];

    roverStatus.Near = 0;
    roverStatus.Search = 0;
  }
  else if(roverStatus.toGoal == 3){
    roverSuccess.Goal_GPS = roverStatus.toGoal;
    roverSuccess.full = 1;
    writeEEPROM(Addr_eeprom,27,(byte)roverSuccess.Goal_GPS);//logger//logger

    roverStatus.Near = 0;
    roverStatus.Search = 0;
    roverMode.Sleep = 1;
  }
  return;
}

//=========Ultrasonic sensor function============================================================================//
unsigned int getUltrasonic_HEAD(){
  long duration;
  digitalWrite(HEAD_Trig,LOW);
  delayMicroseconds(2);
  digitalWrite(HEAD_Trig,HIGH);
  delayMicroseconds(10);
  duration = pulseIn(HEAD_Echo,HIGH);
  return microsecTocm(duration);
}

unsigned int microsecTocm(long microsec){
  return (unsigned int) microsec /29 /2;
}

void filter_angle_search(){
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
  listcm[search_index] = sortBufcm[(int)MEAS_BUF_LEN/2];
  Serial.print(":measure cm:");
  Serial.print(listcm[search_index]);
}

int goal_angle_search(){//探索時、最も測距値が近い角度をゴールの方向と決定する。
  int mincm = listcm[0];
  int minindex = 0;
  for (int i = 0; i < SEAR_BUF_LEN; i++)
  {
    if(0 < listcm[i] && listcm[i] < mincm){
      mincm = listcm[i];
      minindex = i;
    }
  }
  return minindex;
}



//=========LIDAR sensor function============================================================================//
unsigned int getLIDAR(){
  boolean LIDAR_flag = 1;
  int distance;
  int bytenum = 0;
  while (Serial2.available() > 0 && LIDAR_flag == 1)//Near_flagは一時的なもの
  {
    byte c = Serial2.read();
    switch(bytenum){
      case 0://frame header must be 0x59
//        Serial.print("Byte0:");
//        Serial.println(c,HEX);
        if(c == 0x59){
          bytenum += 1;
        }
        break;
      case 1://frame header must be 0x59
//        Serial.print("Byte1:");
//        Serial.println(c,HEX);
        if(c == 0x59){
          bytenum += 1;
        }
        break;
      case 2://distance value low 8 bits
//        Serial.print("Byte2:");
//        Serial.println(c,HEX);
        if(c == 0x59){
          //多分次がcase2
        }
        else{
          distance = (int)c;
          bytenum += 1;
        }
        break;
      case 3://distance value high 8 bits
//        Serial.print("Byte3:");
//        Serial.println(c,HEX);
        distance = distance + 256*(int)c;
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
  if(0<distance && distance < 1000){
    LIDAR_buf = distance;
    return distance;
  }else{
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

void updateGPSlocation(){
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
  Wire.write(0x00); // Normal mode, Sleep duration = 2ms
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

float angle_calculation(){
  x = atan2(yMag - Caliby, xMag - Calibx) / 3.14 * 180 + 180; //磁北を0°(or360°)として出力
  x += Calib;
  x -= 7; //磁北は真北に対して西に（反時計回りに)7°ずれているため、GPSと合わせるために補正をかける

  // calibと7を足したことでcalib+7°~360+calib+7°で出力されてしまうので、0°~360°になるよう調整

  if (x > 360)
  {
    x -=360;
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
  bufx[0] == bufx[CAL_BUF_LEN-1];//先端には0が入っちゃってるのでなんかまともな値を入れる。
  for(int i=0; i<CAL_BUF_LEN; i++) {
    sortBufx[i] = bufx[i];
  }

  //クイックソートで並べ替える
  qsort(sortBufx, CAL_BUF_LEN, sizeof(int), quicksortFunc);
  
  Serial.print(":Min:");
  Serial.print(sortBufx[0]);
  Serial.print(":Max:");
  Serial.print(sortBufx[CAL_BUF_LEN-1]);

  return (sortBufx[1]+sortBufx[CAL_BUF_LEN-2])/2;//取得値ではない「0」が最少と最大になってしまう場合の対処(「0」が複数取れてしまった場合に対応できていないので注意)
}

//Medianフィルタ関数
int ycenter_calculation() {
  //ソート用のバッファ
  static int sortBufy[CAL_BUF_LEN];
  Serial.println("---------------------------------");
  bufy[0] == bufy[CAL_BUF_LEN-1];//先端には0が入っちゃってるのでなんかまともな値を入れる。←効果ないっぽい

  //ソート用バッファにデータをコピー
  for(int i=0; i<CAL_BUF_LEN; i++) {
    sortBufy[i] = bufy[i];
    Serial.print(i);
    Serial.print(",");
    Serial.println(sortBufy[i]);
  }

  int k = 0;
  while(sortBufy[k] == 0){//最初の方に値が入らなかった場合の対応←効果ないっぽい
    sortBufy[k] == sortBufy[CAL_BUF_LEN-1];
    k += 1;
  }

  //クイックソートで並べ替える
  qsort(sortBufy, CAL_BUF_LEN, sizeof(int), quicksortFunc);

  Serial.print(":Min:");
  Serial.print(sortBufy[1]);
  Serial.print(":Max:");
  Serial.print(sortBufy[CAL_BUF_LEN-1]);

  return (sortBufy[1]+sortBufy[CAL_BUF_LEN-2])/2;//取得値ではない「0」が最少と最大になってしまう場合の対処(「0」が複数取れてしまった場合に対応できていないので注意)
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
  if (x < degRtoA){
      delta_theta = degRtoA - x;
      Serial.print(":x < degRtoA:");
      Serial.print(delta_theta);
      
      //閾値内にあるときは真っ直ぐ
      if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= degRtoA)){
        speed_R = Normal_speed;
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print(":Go straight");
        Status_control = 1;//"Go straight"
      }
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else if (threshold/2 < delta_theta && delta_theta <= 180){ 
        speed_R = Normal_speed - (delta_theta * Normal_speed / 180);
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L ); 
        Serial.print(":turn right");
        Status_control = 3;//"turn right"
      }
  
      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else { 
        speed_R = Normal_speed;
        speed_L = Normal_speed - ((360-delta_theta) * Normal_speed / 180);
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print(":turn left");
        Status_control = 2;//"turn left"
      }
  }

  else {
      delta_theta = x - degRtoA;
      Serial.print(":degRtoA < x:");
      Serial.print(delta_theta);
     
      //閾値内にあるときは真っ直ぐ
      if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= 360)){
        speed_R = Normal_speed;
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print(":Go straight");
        Status_control = 1;//"Go straight"
      }
      //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else if (threshold/2 < delta_theta && delta_theta <= 180){ 
        speed_R = Normal_speed;
        speed_L = Normal_speed - (delta_theta * Normal_speed / 180);
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print(":turn left");
        Status_control = 2;//"turn left"
      }
  
      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else { 
        speed_R = Normal_speed - ((360-delta_theta) * Normal_speed / 180);
        speed_L = Normal_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH3, speed_L );
        Serial.print(":turn right");
        Status_control = 3;//"turn right"
        
      }
  }
  Serial.print(":speed_L:");
  Serial.print(speed_L);
  Serial.print(":speed_R:");
  Serial.print(speed_R);
}


void motor_angle_spin()
{    
  if (x < degRtoA){
    delta_theta = degRtoA - x;
    Serial.print(":x < degRtoA:");
    Serial.print(delta_theta);

    if((0 <= delta_theta && delta_theta <= spin_threshold/2)|| (360 - spin_threshold/2 <= delta_theta && delta_theta <= degRtoA)){
      //閾値内にあるときは真っ直ぐ
      if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= degRtoA)){
        speed_R = Slow_speed;
        speed_L = Slow_speed;
        //スロー前進
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite( CH1, speed_R );
        analogWrite( CH2, 0);
        analogWrite( CH3, speed_L );
        analogWrite( CH4, 0);
        Serial.print(":Go straight");
        Status_control = 1;//"Go straight"
        count_forward += 1;
      }
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else if (threshold/2 < delta_theta && delta_theta <= 180){ 
        speed_R = Slow_speed - (delta_theta * Slow_speed / 180);
        speed_L = Slow_speed;
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite( CH1, speed_R );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speed_L ); 
        analogWrite( CH4, 0 );
        Serial.print(":turn right");
        Status_control = 3;//"turn right"
      }
  
      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else { 
        speed_R = Slow_speed;
        speed_L = Slow_speed - ((360-delta_theta) * Slow_speed / 180);
        analogWrite( CH1, speed_R );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speed_L );
        analogWrite( CH4, 0 );
        Serial.print(":turn left");
        Status_control = 2;//"turn left"
        count_spin += 1;
      }
    }
    else{
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      if (threshold/2 < delta_theta && delta_theta <= 180){ 
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, 0);
        analogWrite(CH2, Very_Slow_speed);
        analogWrite(CH3, Very_Slow_speed);
        analogWrite(CH4, 0);
        Serial.print(":spin to right");
        Status_control = 4;//"spin to right"
        count_spin += 1;
      }
  
      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else { 
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, Very_Slow_speed);
        analogWrite(CH2, 0);
        analogWrite(CH3, 0);
        analogWrite(CH4, Very_Slow_speed);
        Serial.print(":spin to left");
        Status_control = 5;//"spin to left
        count_spin += 1;
      }
    }
  }

  else {
    delta_theta = x - degRtoA;
    Serial.print(":degRtoA < x:");
    Serial.print(delta_theta);
    if ((0 <= delta_theta && delta_theta <= spin_threshold/2)|| (360 - spin_threshold/2 <= delta_theta && delta_theta <= 360)){ 
      //閾値内にあるときは真っ直ぐ
      if ((0 <= delta_theta && delta_theta <= threshold/2)|| (360 - threshold/2 <= delta_theta && delta_theta <= 360)){
        speed_R = Slow_speed;
        speed_L = Slow_speed;
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite( CH1, speed_R );
        analogWrite( CH2, 0);
        analogWrite( CH3, speed_L );
        analogWrite( CH4, 0);
        Serial.print(":Go straight");
        Status_control = 1;//"Go straight"
        count_forward += 1;
      }
      //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else if (threshold/2 < delta_theta && delta_theta <= 180){ 
        speed_R = Slow_speed;
        speed_L = Slow_speed - (delta_theta * Slow_speed / 180);
        analogWrite( CH1, speed_R );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speed_L );
        analogWrite( CH4, 0 );
        Serial.print(":turn left");
        Status_control = 2;//"turn left"
      }
  
      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else { 
        speed_R = Slow_speed - ((360-delta_theta) * Slow_speed / 180);
        speed_L = Slow_speed;
        analogWrite( CH1, speed_R );
        analogWrite( CH2, 0 );
        analogWrite( CH3, speed_L );
        analogWrite( CH4, 0 );
        Serial.print(":turn right");
        Status_control = 3;//"turn right"
      }
    }
    else{
      if (threshold/2 < delta_theta && delta_theta <= 180){ 
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, Very_Slow_speed);
        analogWrite(CH2, 0);
        analogWrite(CH3, 0);
        analogWrite(CH4, Very_Slow_speed);
        Serial.print(":spin to left");
        Status_control = 5;//"spin to left
        count_spin += 1;
      }
      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else { 
        digitalWrite(ENABLE, HIGH); // enable
        analogWrite(CH1, 0);
        analogWrite(CH2, Very_Slow_speed);
        analogWrite(CH3, Very_Slow_speed);
        analogWrite(CH4, 0);
        Serial.print(":spin to right");
        Status_control = 4;//"spin to right"
        count_spin += 1;
      }
    }
  }
  Serial.print(":speed_L:");
  Serial.print(speed_L);
  Serial.print(":speed_R:");
  Serial.print(speed_R);
  Serial.print(":count_forward:");
  Serial.print(count_forward);
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

void EEPROM_write_int(int addr_device, unsigned int addr_res, int data){
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
//    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res+i, p[i]);
  }
//  Serial.println("");
}


void EEPROM_write_long(int addr_device, unsigned int addr_res, unsigned long data){
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
//    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res+i, p[i]);
  }
//  Serial.println("");
}


void EEPROM_write_float(int addr_device, unsigned int addr_res, double data){
  unsigned char *p = (unsigned char *)&data;
  int i;
  for (i = 0; i < (int)sizeof(data); i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
//    Serial.println(p[i]);
    writeEEPROM(addr_device, addr_res+i, p[i]);
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
 
  Wire.requestFrom(addr_device,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


double EEPROM_read_float(int addr_device, unsigned int addr_res){
  unsigned char p_read[4];
  for (int i = 0; i < 4; i++){
//    Serial.print(i+1);
//    Serial.print("th byte:");
    p_read[i] = readEEPROM(addr_device, addr_res+i);
//    Serial.println(p_read[i]);
  }
//  Serial.println("");
  double *d = (double *)p_read;
  double data = *d;
  return data;
}

void LogToEEPROM(){
    EEPROM_write_int(Addr_eeprom, DATA_ADDRESS,xMag);
    DATA_ADDRESS += 2;
    EEPROM_write_int(Addr_eeprom, DATA_ADDRESS,yMag);
    DATA_ADDRESS += 2;
    EEPROM_write_int(Addr_eeprom, DATA_ADDRESS,Calibx);
    DATA_ADDRESS += 2;
    EEPROM_write_int(Addr_eeprom, DATA_ADDRESS,Caliby);
    DATA_ADDRESS += 2;
    EEPROM_write_float(Addr_eeprom, DATA_ADDRESS,x);
    DATA_ADDRESS += 4;
    EEPROM_write_int(Addr_eeprom, DATA_ADDRESS,cm_long);
    DATA_ADDRESS += 2;
    EEPROM_write_float(Addr_eeprom, DATA_ADDRESS,LatR);
    DATA_ADDRESS += 4;
    EEPROM_write_float(Addr_eeprom, DATA_ADDRESS,LngR);
    DATA_ADDRESS += 4;
    EEPROM_write_float(Addr_eeprom, DATA_ADDRESS,degRtoA);
    DATA_ADDRESS += 4;
    writeEEPROM(Addr_eeprom, DATA_ADDRESS,(byte)Status_control);
    DATA_ADDRESS += 2;
    time = millis();
    EEPROM_write_long(Addr_eeprom, DATA_ADDRESS,time);
    DATA_ADDRESS += 4;
}

void LogGPSdata(){
  EEPROM_write_float(Addr_eeprom, 0, dataRx.gpsData.latA[0]);
  EEPROM_write_float(Addr_eeprom, 4, dataRx.gpsData.lngA[0]);
  EEPROM_write_float(Addr_eeprom, 8, dataRx.gpsData.latA[1]);
  EEPROM_write_float(Addr_eeprom, 12, dataRx.gpsData.lngA[1]);
  EEPROM_write_float(Addr_eeprom, 16, dataRx.gpsData.latA[2]);
  EEPROM_write_float(Addr_eeprom, 20, dataRx.gpsData.lngA[2]);
}
   
//============SDCard function=========================================================================//
void LogToSDCard(){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.print(time);
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
    dataFile.print(Status_control);
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

void  writeToTwelite (){      
  int ctr1 = 0;
  readRoverData();
  encodeCyclic();
  Serial2.print(":000100");
  //Serial.print(":000100");
  while (ctr1<2*sizeof(roverData)){
    if((uint8_t)encodedTx[ctr1]<16){
      Serial2.print("0");
      //Serial.print("0");
    }
    Serial2.print(encodedTx[ctr1],HEX);
    //Serial.print(encodedTx[ctr1],HEX);
    ctr1++;
  }
  Serial2.print("X\r\n");
  //Serial.print("X\r\n");
}

void readRoverData(){
  packetTx.message.roverComsStat = 4;
  Serial.print("roverComsStat:");
  Serial.println(packetTx.message.roverComsStat);
  packetTx.message.xMag = xMag;
  Serial.print("xMag:");
  Serial.println(packetTx.message.xMag);
  packetTx.message.yMag= yMag;
  Serial.print("yMag:");
  Serial.println(packetTx.message.yMag);
  packetTx.message.calibX= Calibx;
  Serial.print("Calibx:");
  Serial.println(packetTx.message.calibX);
  packetTx.message.calibY= Caliby;
  Serial.print("Caliby:");
  Serial.println(packetTx.message.calibY);
  packetTx.message.x= x;
  Serial.print("x:");
  Serial.println(packetTx.message.x);
  packetTx.message.cmLong = cm_LIDAR;
  Serial.print("cm_long:");
  Serial.println(packetTx.message.cmLong);
  packetTx.message.latR= LatR;
  Serial.print("LatR:");
  Serial.println(packetTx.message.latR);
  packetTx.message.lngR= LngR;
  Serial.print("LngR:");
  Serial.println(packetTx.message.lngR);
  packetTx.message.degRtoA= degRtoA;
  Serial.print("degRtoA:");
  Serial.println(packetTx.message.degRtoA);
  packetTx.message.statusControl= Status_control;
  Serial.print("Status_control:");
  Serial.println(packetTx.message.statusControl);
  packetTx.message.time= time;
  Serial.print("time:");
  Serial.println(packetTx.message.time);
}

void encodeCyclic() {
  uint8_t ctr = 0;
  uint8_t m;
  while(ctr<sizeof(roverData)) {
    m = packetTx.packetData[ctr]>>4;
    encodedTx[2*ctr] = ((m&1)*generator[3])^(((m>>1)&1)*generator[2])^
                        (((m>>2)&1)*generator[1])^(((m>>3)&1)*generator[0]);
    //Serial.print(encodedTx[2*ctr],HEX);
    m = packetTx.packetData[ctr];
    encodedTx[2*ctr+1] = ((m&1)*generator[3])^(((m>>1)&1)*generator[2])^
                        (((m>>2)&1)*generator[1])^(((m>>3)&1)*generator[0]);
    //Serial.println(encodedTx[2*ctr+1],HEX);
    ctr++;
  }
}

bool checkError(uint8_t dataByte) {
  uint8_t p[3];
  uint8_t ctr = 0;
  p[0] = dataByte&parityCheck[0];
  p[1] = dataByte&parityCheck[1];
  p[2] = dataByte&parityCheck[2];
  while(ctr<sizeof(gpsDataStruct)/2) {
    p[0] = (p[0]&1)^(p[0]>>1);
    p[1] = (p[1]&1)^(p[1]>>1);
    p[2] = (p[2]&1)^(p[2]>>1);
    ctr++;
  }
  return (p[0]>0)||(p[1]>0)||(p[2]>0);
}

void decodeCyclic() {
  uint8_t ctr = 0;
  bool error[2];
  while (ctr<sizeof(gpsDataStruct)) {
    encodedRx[2*ctr] = encodedRx[2*ctr]&0x7F;
    encodedRx[2*ctr+1] = encodedRx[2*ctr+1]&0x7F;
    error[0] = checkError(encodedRx[2*ctr]);
    error[1] = checkError(encodedRx[2*ctr+1]);
    dataRx.gpsBytes[ctr] = ((encodedRx[2*ctr]<<1)&0xF0)+
                            ((encodedRx[2*ctr+1]>>3)&0x0F);//populate GPS data of goals in dataRx     
    if(error[0]||error[1]) { //NACK
        Serial2.print(":000102X\r\n");
        return true;
    }
    ctr++;
  }
  //If no errors send ACK
  Serial2.print(":000101X\r\n");
  return false;
}
