#include <Servo.h>
Servo myservo;
const int SV_PIN = 7;       // サーボモーターをデジタルピン7に
int degree = 0;     // PWMパルス幅を1450マイクロ秒(2.4ms)に設定
boolean reverse =0;

void setup(){
 
   myservo.attach(SV_PIN, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
  
}
 
void loop(){
 
   myservo.write(degree);    // サーボモーターを0度の位置まで動かす
   if(reverse == 0){
    degree += 1;
   }
   else{
    degree -= 1;
   }
   delay(500);
   if(degree > 30){
    reverse = 1;
   }
   if(degree < 1){
    reverse = 0;
   }
}
