#include "./Motor.h"
Motor motor(3,4,5,6,7);
const int nominalSpeed = 250;
const int slowSpeed = 200;
const int verySlowSpeed = 150;

void setup() {
  // put your setup fcode here, to run once:
  Serial.begin(9600);
  motor.init();
  delay(500);
}

void loop() {
//    Serial.println("1:Normal rotaion");
//    digitalWrite(ENABLE,HIGH); // enable on
////    digitalWrite(CH1,HIGH);
//    analogWrite(CH1,1*255);    
////    digitalWrite(CH2,LOW); 
//    digitalWrite(CH3,1*255);    
//    digitalWrite(CH4,LOW); 
//    delay(4000);
////
//    Serial.println("0:STOP");
//    digitalWrite(ENABLE,LOW); // disable
//    delay(1000);
//    
//    Serial.println("2:Reverse rotation");
//    digitalWrite(ENABLE,LOW); // enable on
//    delay(100);                // 低速で回転させるための調整時間
//    digitalWrite(ENABLE,HIGH); // enable on
//    analogWrite(CH2,1*255);    
//    digitalWrite(CH1,LOW); 
//    digitalWrite(CH4,1*255);    
//    digitalWrite(CH3,LOW); 
//    delay(4000);
    
//    Serial.println("0:STOP");
//    digitalWrite(ENABLE,LOW); // disable
//    delay(1000);

    //motor.goStraight(100);
    motor.left(200);
    delay(4000);
    motor.stop();
    delay(1000);
}
