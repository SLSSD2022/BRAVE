void setup() {
  Serial.begin(9600); //9600bpsでシリアルポートを開く
}

void loop() {                                                    //{}内を無限ループで実行する
  delay(1000);                                                  //1000ms(10秒)毎に計算
  float LatR = 35.713897, LongR = 139.760474;       //到着時の自分の緯度経度
  float LatA =35.713844, LongA = 139.760437;       //目的地の緯度経度
  
  Serial.println("Calculation start");     
  Serial.print("The direction of Rover and A is = ");                               //目的地Aの方角(°）
  Serial.print(atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180);
  Serial.print("deg: The distance between Rover and A is= ");                             //目的地A迄の距離(m)
  int RtoA = sqrt(pow(LongR - LongA, 2) + pow(LatR - LatA, 2)) * 99096.44;
  Serial.print(RtoA);
  Serial.println("m");
  delay(1000);
}

  
