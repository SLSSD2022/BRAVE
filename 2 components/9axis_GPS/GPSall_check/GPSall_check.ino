#include <TinyGPSPlus.h>
TinyGPSPlus gps;

float LatA = 35.7100069, LongA = 139.8108103;  //目的地Aの緯度経度(今回はスカイツリー)
float LatR, LongR;

float degRtoA; //目的地とGPS現在地の角度
float delta_theta;
int threshold = 30; //角度の差分の閾値

boolean GPS_flag = 1;

//int buf_degRtoA[BUF_LEN];
//int index_degRtoA = 0;

float deg2rad(float deg) {
        return deg * PI / 180.0;
    }


void setup()
{
  // デバッグ用シリアル通信は9600bps
  Serial.begin(9600);
  Serial1.begin(9600);
  delay(500);
}


void loop()
{

  //---------------------GPS取得--------------------------------------------------
  while (Serial1.available() > 0 && GPS_flag == 1){
//    Serial.print("YES");
    char c = Serial1.read();
    Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated()){
      Serial.println("");
      Serial.println("I got it!");
      LatR = gps.location.lat(); //roverの緯度を計算
      LongR = gps.location.lng(); //roverの経度を計算
      degRtoA = atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180;
      Serial.println("");
      Serial.println("-------------------");
      Serial.println(degRtoA);
      Serial.println("-------------------");
      Serial.println("");
      GPS_flag = 0;
    }
    //連続した次の文字が来るときでも、間が空いてしまう可能性があるのでdelayを挟む
    delay(1);
  } 
  Serial.println("");
  Serial.println("I cannot get it........");
  //---------------------GPS取得--------------------------------------------------
  delay(200);
  GPS_flag = 1;
}
