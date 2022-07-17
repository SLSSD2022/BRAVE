//Header file
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "./rover.h"
#include "./Motor.h"
#include "./IMU.h"
//------------------------------Motor------------------------------
Motor motor(3,5,4,6,7);
const int Threshold = 10;
const int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)
const int nominalSpeed = 150;
const int slowSpeed = 100;
const int verySlowSpeed = 70;

//9axis filter
//------------------------------9axis sensor------------------------------
IMU imu(0.00, 0.00, 0.00, 0, 0, 0, 265, 20, 132);
//キャリブレーション用バッファの長さ
#define CAL_BUF_LEN 100
int bufx[CAL_BUF_LEN];
int bufy[CAL_BUF_LEN];
int calIndex = 0;
boolean calibxyFlag = 0;
int deltaN = 0;
int nextCalib = 0;
//------------------------------SD card------------------------------
const int chipSelect = 53;
const int SDSW = 49;


//------------------------------Control Status----------------------------
Rover rover;

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
  Serial.println("===========================================================");
  Serial.println("=====Today, I'm going to calibrate my 9 axis sensor!!!=====");
  Serial.println("===========================================================");
  //setting status&environment
  rover.status.calibrated = 0;
  rover.printAll();

  //initialization
  Serial.println("");
  Serial.println("------------------ Initialization!!! ------------------");
  motor.init();motor.setThreshold(Threshold,spinThreshold);
  imu.init();
  SDinit();

  //バッファの初期化
  for (int i = 0; i < CAL_BUF_LEN; i++) {
    bufx[i] = 0;
    bufy[i] = 0;
  }
  SDprint("datalog.txt","2022/7/17(likely) Calibration");
  Serial.println("------------------ Mission Start!!! ------------------");
}


/*
  ############################################################################################################
  ############################################################################################################
*/


void loop()
{
  //---------------------9軸取得--------------------------------------------------
  // BMX055 ジャイロの読み取り
  imu.getGyro();
  // BMX055 磁気の読み取り
  imu.getMag();

  //キャリブレーションが終了しているなら
  if (calibxyFlag == 1) {
    rover.data.x = imu.angleCalculation();
  }

  //---------------------Check parameter & update Status--------------------------------------------------
 
  imu.printAll();
//  rover.data.printAll();

  //---------------------Special control for each status------------------------------------------------------


  //calibration
  if (calibxyFlag == 0) {
    calibLoop();
    Serial.print(":calibrationxy");
  }
  else{
    calibNorth();
    Serial.print(":calibrationNorth");
  }

  //---------------------Motor Control--------------------------------------------------
  if (calibxyFlag == 0) { //キャリブレーション時
    motor.spinLeft(slowSpeed);
  }
  else{
    motor.stop();
  }
  rover.data.motorControl = motor.controlStatus;

  //---------------------Logger------------------------------------------------------
  rover.data.overallTime = millis();//it's good if time is synchronized with GPStime
  LogToSDCard();
  //---------------------Communication(sending HK for every 10 seconds)------------------------------------------------------


  //最後にシリアル通信を改行する
  Serial.println("");
}

/*
  ############################################################################################################
  ############################################################################################################
*/



//=========Status control function============================================================================//

void calibLoop(){
  Serial.print(":calIndex:");
  Serial.print(calIndex);
  bufx[calIndex] = imu.xMag;
  bufy[calIndex] = imu.yMag;
  if (calIndex == CAL_BUF_LEN - 1) { //バッファに値がたまったら
    imu.calibx = xcenter_calculation();
    imu.caliby = ycenter_calculation();
    calibxyFlag = 1;
    Serial.print(":calib_x:");
    Serial.print(imu.calibx);
    Serial.print(":calib_y:");
    Serial.print(imu.caliby);
    rover.data.x = imu.angleCalculation();//このループ後半のためだけ
  }
  else {
    calIndex = (calIndex + 1) % CAL_BUF_LEN;
  }
  return;
}


void calibNorth(){
  //磁北の反対方向にローバーを向かせていると仮定する
  //xが示すべき値は-7。計算されたx出力の差をdeltaNとする。
  deltaN = -7 - rover.data.x;
  //deltaNをcalibに加えて更新する。
  deltaN = deltaN % 360;
  if (deltaN < 0)
  {
    deltaN += 360;
  }
  Serial.print(":deltaN:");
  Serial.print(deltaN);
  nextCalib = imu.calib + deltaN;
  //0°~360°になるよう調整
  nextCalib = nextCalib % 360;
  if (nextCalib < 0)
  {
    nextCalib += 360;
  }
  Serial.print(":nextCalib:");
  Serial.print(nextCalib);
//  imu.calib = nextCalib;
  return;
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


//============SDCard function=========================================================================//
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
    }
    else {
      Serial.println("Card not inserted!");
//      break;
    }
    delay(1000);
  }
  
}


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
    dataFile.print(imu.calib);
    dataFile.print(",");
    dataFile.print(rover.data.x);
    dataFile.print(",");
    dataFile.print(deltaN);
    dataFile.print(",");
    dataFile.print(nextCalib);
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
