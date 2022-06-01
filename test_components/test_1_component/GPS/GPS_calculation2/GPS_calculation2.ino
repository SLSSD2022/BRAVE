#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
 
TinyGPSPlus gps;
SoftwareSerial mySerial(2, 3); // RX, TX

void setup() {
 // Open serial communications and wait for port to open:
 Serial.begin(115200);
 while (!Serial) {
 ; // wait for serial port to connect. Needed for native USB port only
 }
 
 //Serial.println("Goodnight moon!");
 
 // set the data rate for the SoftwareSerial port
 mySerial.begin(9600);
 //mySerial.println("Hello, world?");
}

void loop() {
 while (mySerial.available() > 0){
 char c = mySerial.read();
 //Serial.print(c);
 gps.encode(c);
 if (gps.location.isUpdated()){
 float LatR = gps.location.lat();
 float LongR = gps.location.lng();
 Serial.print("ROVER'S LAT ="); Serial.println(gps.location.lat(), 6);
 Serial.print("Rover's LONG="); Serial.println(gps.location.lng(), 6);
 
 float LatA = 35.71531204697, LongA = 139.7606595037;       //目的地Aの緯度経度(今回は工学部7号館)
 float LatB = 35.71342444118, LongB = 139.7619808394;       //目的地Bの緯度経度(今回は中央食堂)
 float LatC = 35.71352735252, LongC = 139.7623036554;       //目的地Cの緯度経度(今回は安田講堂)
 Serial.println("Calculation start");     
 Serial.print("The direction of Rover and A is = ");                               //目的地Aの方角(°）
 Serial.print(atan2((LongR - LongA) * 1.23, (LatR - LatA)) * 57.3 + 180);
 Serial.print("deg: The distance between Rover and A is= ");                             //目的地A迄の距離(m)
 int RtoA = sqrt(pow(LongR - LongA, 2) + pow(LatR - LatA, 2)) * 99096.44;
 Serial.print(RtoA);
 Serial.println("m");

 Serial.print("The direction of Rover and B is = ");                               //目的地Bの方角(°）
 Serial.print(atan2((LongR - LongB) * 1.23, (LatR - LatB)) * 57.3 + 180);
 Serial.print("deg: The distance between Rover and B is= ");                             //目的地B迄の距離(m)
 int RtoB = sqrt(pow(LongR - LongB, 2) + pow(LatR - LatB, 2)) * 99096.44;
 Serial.print(RtoB);
 Serial.println("m");

 Serial.print("The direction of Rover and C is = ");                               //目的地Cの方角(°）
 Serial.print(atan2((LongR - LongC) * 1.23, (LatR - LatC)) * 57.3 + 180);
 Serial.print("deg: The distance between Rover and C is= ");                             //目的地C迄の距離(m)
 int RtoC = sqrt(pow(LongR - LongC, 2) + pow(LatR - LatC, 2)) * 99096.44;
 Serial.print(RtoC);
 Serial.println("m");

 if (RtoA < RtoB && RtoA < RtoC){
    Serial.print("A is the shortest path");
 }

 else if (RtoB < RtoA && RtoB < RtoC){
    Serial.print("B is the shortest path");
 }
  
 else if (RtoC < RtoA && RtoC < RtoB){
    Serial.print("C is the shortest path");
 }
 delay(10000); 
}
 }
 
 }
 
