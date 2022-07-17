#include <Servo.h>
Servo myservo;
const int SV_PIN = 23;       // サーボモーターをデジタルピン7に
int degree = 150;     // PWMパルス幅を1450マイクロ秒(2.4ms)に設定
boolean reverse = 0;

void setup(){
 
   myservo.attach(SV_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
   myservo.write(degree);    // サーボモーターを0度の位置まで動かす
  
}
 
void loop(){
  
//   myservo.write(degree);    // サーボモーターを0度の位置まで動かす
//   if(reverse == 0){
//    degree += 1;
//   }
//   else{
//    degree -= 1;
//   }
//   delay(20);
//   if(degree > 90){
//    reverse = 1;
//   }
//   if(degree < 10){
//    reverse = 0;
//   }
}
