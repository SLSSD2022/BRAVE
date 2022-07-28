
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
int listdeg[SEAR_BUF_LEN];
int searchIndex = 0;


/*
  ############################################################################################################
  ############################################################################################################
*/


void toGoalLoop(){
//  int stic = millis();
  //---------------------9軸取得--------------------------------------------------
  // BMX055 ジャイロの読み取り
  imu.getGyro();
  // BMX055 磁気の読み取り
  imu.getMag();
  rover.data.xMag = imu.xMag;
  rover.data.yMag = imu.yMag;
//  Serial.print(imu.xMag);Serial.print(",");Serial.print(imu.yMag);//
  rover.data.calibx = imu.calibx;
  rover.data.caliby = imu.caliby;
//  toc = millis();
//  Serial.print(":getIMU:");Serial.print(toc - tic);
//  tic = toc;

  //キャリブレーションが終了しているなら
  if (rover.status.calibrated == 1) {
    rover.data.x = imu.angleCalculation();
  }
//  toc = millis();
//  Serial.print(":getx:");Serial.print(toc - tic);
//  tic = toc;

  //---------------------LIDARセンサ取得--------------------------------------------------
//  rover.data.cmLidar = lidar.getDistance();

  //---------------------超音波(短・前面)取得--------------------------------------------------
  //rover.data.cmHead = ultrasonicHead.getDistance();

  //---------------------超音波(長・前面)取得--------------------------------------------------
  //rover.data.cmLong = ultrasonicLong.getDistance();

  //---------------------GPS acquisition--------------------------------------------------
  gps.trycatchGPSlocation(&rover.data.latR,&rover.data.lngR);
  rover.data.degRtoA = atan2((rover.data.lngR - rover.data.lngA) * 1.23, (rover.data.latR - rover.data.latA)) * 57.3 + 180;
  rover.data.rangeRtoA = gps.distanceBetween(rover.data.latR, rover.data.lngR, rover.data.latA, rover.data.lngA);
//  toc = millis();
//  Serial.print(":getGPS:");Serial.print(toc - tic);
//  tic = toc;

  //---------------------Check parameter & update Status--------------------------------------------------
 
  imu.printAll();
  rover.data.printAll();

  if (rover.data.rangeRtoA > 1.0) {
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
    //emergencyStopFlag = 1;
  } else {
    emergencyStopFlag = 0;
  }

  //---------------------Special control for each status------------------------------------------------------


  //calibration
  if (rover.status.calibrated == 0) {
    calibLoop();
  }

  //ゴール探索時
  if (rover.status.calibrated == 1 && rover.status.near == 1) {
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
  else if (emergencyStopFlag == 0 && rover.status.calibrated == 0) { //キャリブレーション時
    motor.spinLeft(slowSpeed);
    Serial.print(":calibration");
  }
  else if (emergencyStopFlag == 0 && rover.status.calibrated == 1 && rover.status.near == 1) { //ゴール5m付近時
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
  else if (emergencyStopFlag == 0 && rover.status.calibrated == 1 && rover.status.near == 0 ) { //通常走行時
    motor.angleGo(rover.data.x,rover.data.degRtoA,nominalSpeed);
  }
  rover.data.motorControl = motor.controlStatus;
//  toc = millis();
//  Serial.print(":motorcontrol:");Serial.print(toc - tic);
//  tic = toc;

  //---------------------Logger------------------------------------------------------
  rover.data.overallTime = millis();//it's good if time is synchronized with GPStime
  LogToSDCard();
//  toc = millis();
//  Serial.print(":SDcard:");Serial.print(toc - tic);
//  tic = toc;
//  if (memoryFlag > 5) {
//    eeprom.log();
//    memoryFlag = 0;
//  }
//  else {
//    memoryFlag += 1;
//  }
//  toc = millis();
//  Serial.print(":EEPROM:");Serial.print(toc - tic);
//  tic = toc;

  //---------------------Communication(sending HK for every 10 seconds)------------------------------------------------------

  stop = millis();
  int timer = stop - start;
  
//  Serial.print(":stop");
//  Serial.print(stop);
//  Serial.print(":start");
//  Serial.print(start);
//  Serial.print(":timer");
//  Serial.println(timer);

  if (timer > 5000) {
    Serial.println(":Communication start!");
    comm.HKtoGS(&imu,&rover.data);
    start = millis();
    Serial.println(":Communication end!");
  }
//  toc = millis();
//  Serial.print(":comm(end):");Serial.print(toc - tic);
//  tic = toc;
  
  //---------------------ステータス更新--------------------------------------------------
//  int stoc = millis();
//  Serial.print(":looptime:");Serial.print(stoc -stic);
  //最後にシリアル通信を改行する
  Serial.println("");
}



/*
  ############################################################################################################
  ############################################################################################################
*/



//=========Status control function============================================================================//
void successManagement() {
  motor.stop();
  if (rover.status.toGoal < 3) {
    rover.success.goalGPS = rover.status.toGoal;
    eeprom.write(27, (byte)rover.success.goalGPS); //logger
    SDprintln("datalog.txt","Achieved one goal!");
    SDprintln("datalog.txt",rover.status.toGoal);

    int next = goalRoute[rover.status.toGoal];//set next goal to the destination
    rover.status.toGoal += 1;
    rover.data.latA = comm.gpsPacket.gpsData.latA[next];
    rover.data.lngA = comm.gpsPacket.gpsData.lngA[next];
    
    rover.status.near = 0;
    rover.status.search = 0;
    comm.updateGoalStat(); // Increment "aim to goal" for moving to next
  }
  else if (rover.status.toGoal == 3) {
    rover.success.goalGPS = rover.status.toGoal;
    rover.success.full = 1;
    eeprom.write(27, (byte)rover.success.goalGPS); //logger//logger
    SDprintln("datalog.txt","Achieved all goal!");
    SDprintln("datalog.txt",rover.status.toGoal);

    rover.status.near = 0;
    rover.status.search = 0;
    rover.mode.sleep = 1;
  }
  globalFile.close();
  delay(3000);
  Serial.println("AHA");
  myCAMSaveToSDFile();
  Serial.println("UHU");
  globalFile = SD.open("datalog.txt", FILE_WRITE);
  if (globalFile) {
  }
  // if the file isn't open, pop up an error:
  else {
//      Serial.println("error opening datalog.txt");
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
  Serial.print(":calIndex:");
  Serial.print(calIndex);
  bufx[calIndex] = imu.xMag;
  bufy[calIndex] = imu.yMag;
  if (calIndex == CAL_BUF_LEN - 1) { //バッファに値がたまったら
    imu.calibx = xcenter_calculation();
    imu.caliby = ycenter_calculation();
    rover.status.calibrated = 1 ;
    Serial.print(":calib_x:");
    Serial.print(imu.calibx);
    Serial.print(":calib_y:");
    Serial.print(imu.caliby);
    rover.data.x = imu.angleCalculation();//このループ後半のためだけ
    calIndex = 0;
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
  if (rover.data.x < rover.data.degRtoA) {
    deltaTheta = rover.data.degRtoA - rover.data.x;
    Serial.print(":x < degRtoA:");
    Serial.print(deltaTheta);

    if ((0 <= deltaTheta && deltaTheta <= motor.spinThreshold / 2) || (360 - motor.spinThreshold / 2 <= deltaTheta && deltaTheta <= rover.data.degRtoA)) {
      //閾値内にあるときは真っ直ぐ
      if ((0 <= deltaTheta && deltaTheta <= motor.threshold / 2) || (360 - motor.threshold / 2 <= deltaTheta && deltaTheta <= rover.data.degRtoA)) {
        motor.goStraight(slowSpeed);
        forwardCount += 1;
      }
      //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else if (motor.threshold / 2 < deltaTheta && deltaTheta <= 180) {
        motor.turn(slowSpeed,slowSpeed - (deltaTheta * slowSpeed / 180));//"turn right"
      }

      //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
      else {
        motor.turn(slowSpeed - ((360 - deltaTheta) * slowSpeed / 180),slowSpeed);//"turn left"
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
    deltaTheta = rover.data.x - rover.data.degRtoA;
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
        motor.turn(slowSpeed - (deltaTheta * slowSpeed / 180),slowSpeed);//"turn left"
      }

      //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
      else {
        motor.turn(slowSpeed,slowSpeed - ((360 - deltaTheta) * slowSpeed / 180));//"turn right"
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
  Serial.print(motor.powerL);
  Serial.print(":powerR:");
  Serial.print(motor.powerR);
  Serial.print(":forwardCount:");
  Serial.print(forwardCount);
}

//============SDCard function=========================================================================//

void LogToSDCard() {
  sdstop = millis();
  int timer = sdstop - sdstart;
  if(timer > 5000)
  {
    Serial.println("--------------Going to SDopenclose...---------------");
    globalFile.print(rover.data.xMag);
    globalFile.print(",");
    globalFile.print(rover.data.yMag);
    globalFile.print(",");
    globalFile.print(rover.data.calibx);
    globalFile.print(",");
    globalFile.print(rover.data.caliby);
    globalFile.print(",");
    globalFile.print(rover.data.x);
    globalFile.print(",");
    globalFile.print(rover.data.cmHead);
    globalFile.print(",");
    globalFile.print(rover.data.cmLong);
    globalFile.print(",");
    globalFile.print(rover.data.cmLidar);
    globalFile.print(",");
    globalFile.print(rover.data.latA,6);
    globalFile.print(",");
    globalFile.print(rover.data.lngA,6);
    globalFile.print(",");
    globalFile.print(rover.data.latR,6);
    globalFile.print(",");
    globalFile.print(rover.data.lngR,6);
    globalFile.print(",");
    globalFile.print(rover.data.degRtoA);
    globalFile.print(",");
    globalFile.print(rover.data.rangeRtoA);
    globalFile.print(",");
    globalFile.print(rover.data.motorControl);
    globalFile.println("");
    globalFile.close();
//    toc = millis();
//    Serial.print(":closeSDcard:");Serial.print(toc - tic);
//    tic = toc;
    globalFile = SD.open("datalog.txt", FILE_WRITE);
//    toc = millis();
//    Serial.print(":openSDcard:");Serial.print(toc - tic);
//    tic = toc;
    if (globalFile) {
    }
    // if the file isn't open, pop up an error:
    else {
//      Serial.println("error opening datalog.txt");
    }
    sdstart = millis();
  }
  else
  {
    globalFile.print(rover.data.xMag);
    globalFile.print(",");
    globalFile.print(rover.data.yMag);
    globalFile.print(",");
    globalFile.print(",");
    globalFile.print(",");
    globalFile.print(rover.data.x);
    globalFile.print(",");
    globalFile.print(",");
    globalFile.print(",");
    globalFile.print(",");
    globalFile.print(",");
    globalFile.print(",");
    if(gps.updateFlag == 1){
      globalFile.print(rover.data.latR,6);
      globalFile.print(",");
      globalFile.print(rover.data.lngR,6);
      globalFile.print(",");
    }
    else{
      globalFile.print(",");
      globalFile.print(",");
    }
    globalFile.print(rover.data.degRtoA);
    globalFile.print(",");
    globalFile.print(rover.data.rangeRtoA);
    globalFile.print(",");
    globalFile.print(rover.data.motorControl);
    globalFile.println("");
  }
//  sdbuf[sdbufIndex] = rover.data;
//  sdbufIndex += 1;
//  if(sdbufIndex == SD_BUF_LEN)
//  {
//    File dataFile = SD.open("datalog.txt", FILE_WRITE);
//    toc = millis();
//    Serial.print(":openSDcard:");Serial.print(toc - tic);
//    tic = toc;
//    if (dataFile) {
//      for(int i = 0;i < SD_BUF_LEN;i++){
//        dataFile.print(sdbuf[i].xMag);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].yMag);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].calibx);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].caliby);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].x);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].cmHead);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].cmLong);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].cmLidar);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].latA,10);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].lngA,10);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].latR,10);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].lngR,10);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].degRtoA);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].rangeRtoA);
//        dataFile.print(",");
//        dataFile.print(sdbuf[i].motorControl);
//        dataFile.println("");
//      }
//      toc = millis();
//      Serial.print(":writealldata:");Serial.print(toc - tic);
//      tic = toc;
//      dataFile.close();
//      toc = millis();
//      Serial.print(":closeSDcard:");Serial.print(toc - tic);
//      tic = toc;
//    }
//    // if the file isn't open, pop up an error:
//    else {
//      Serial.println("error opening datalog.txt");
//    }
//    sdbufIndex = 0;
//  }
}



void myCAMSaveToSDFile(){
  char str[8];
  byte buf[256];
  static int i = 0;
  static int k = 0;
  uint8_t temp = 0,temp_last=0;
  uint32_t length = 0;
  bool is_header = false;
  File outFile;
  //Flush the FIFO
  myCAM.flush_fifo();
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start Capture"));
  while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
  Serial.println(F("Capture Done."));  
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //384K
  {
    Serial.println(F("Over size."));
    return ;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return ;
  }
  //Construct a file name
  k = k + 1;
  itoa(k, str, 10);
  strcat(str, ".jpg");
  //Open the new file
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if(!outFile){
    Serial.println(F("File open faild"));
    return;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9     
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);    
      //Close the file
      outFile.close();
      Serial.println(F("Image save OK."));
      is_header = false;
      i = 0;
    }  
    if (is_header == true)
    { 
      //Write image data to buffer if not full
      if (i < 256)
      buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }        
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;   
    } 
  } 
}
