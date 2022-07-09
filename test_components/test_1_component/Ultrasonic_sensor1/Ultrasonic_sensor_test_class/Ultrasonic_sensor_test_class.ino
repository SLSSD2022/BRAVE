//#define HEADpin A15
#include "./Ultrasonic.h"
Ultrasonic sensor(22,24);
long anVolt, cm;



void setup() {

   Serial.begin(9600);

}



 
void loop() { 

   //ultrasonicLong.getDistance();
   Serial.println(sensor.distance);

   delay(100);

}
