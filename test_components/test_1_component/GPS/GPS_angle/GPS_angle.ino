#include <TinyGPSPlus.h>
 
TinyGPSPlus gps;
float LatA = 35.7100069, LongA = 139.8108103;  //目的地Aの緯度経度(今回はスカイツリー)


void setup() {
 // Open serial communications and wait for port to open:
 Serial.begin(9600);
 while (!Serial) {
 ; // wait for serial port to connect. Needed for native USB port only
 }

 Serial1.begin(9600);
 while (!Serial1) {
 ; // wait for serial port to connect. Needed for native USB port only
 }
 
}

void loop() {
  while (Serial1.available() > 0){
    char c = Serial1.read();
    //Serial.print(c);
    gps.encode(c);
    //GPSが更新されたら、ローバーの緯度経度からローバーと目的地の真北を基準にした角度のずれを計算
    if (gps.location.isUpdated()){
      float LatR = gps.location.lat();
      float LongR = gps.location.lng();
      Serial.print("LAT ="); Serial.print(gps.location.lat(), 10);
      Serial.print(" : LONG="); Serial.print(gps.location.lng(), 10);
      //Serial.print(atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180);
     
      float degRtoA = atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180;
      Serial.print(" : degRtoA=");Serial.println(degRtoA);
      //degRtoAはroverと目的地Aの真北を基準にした角度のずれ
    }
  }
}
 
