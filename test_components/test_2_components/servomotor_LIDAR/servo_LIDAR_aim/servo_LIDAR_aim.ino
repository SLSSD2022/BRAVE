#include <Servo.h>
Servo servo_YAW;
Servo servo_PITCH;
const int SV_YAW = 24;       // サーボモーターをデジタルピン7に
const int SV_PITCH = 7;       // サーボモーターをデジタルピン7に
int deg_YAW = 73;     // PWMパルス幅を1450マイクロ秒(2.4ms)に設定
int deg_PITCH = 8;     // PWMパルス幅を1450マイクロ秒(2.4ms)に設定

//LIDARセンサー
int bytenum = 0;
int cm_LIDAR = 0;
boolean LIDAR_flag = 1;
int LIDAR_buf = 0;
int distance = 0;
int count = 0;

void setup(){
  Serial.begin(9600);
 
  servo_YAW.attach(SV_YAW, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
  servo_YAW.write(deg_YAW);    // サーボモーターを0度の位置まで動かす
  servo_PITCH.attach(SV_PITCH, 500, 2400);  // サーボの割当(パルス幅500~2400msに指定)
  servo_PITCH.write(deg_PITCH);    // サーボモーターを0度の位置まで動かす

  
  //TWElite通信用ハードウェアシリアル
  Serial2.begin(115200);
  while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
}
 
void loop(){
//  servo_YAW.write(deg_YAW);
//  servo_PITCH.write(deg_PITCH);
  //---------------------LIDARセンサ取得--------------------------------------------------
  while (Serial2.available() > 0) {
    byte c = Serial2.read();
    switch(bytenum){
      case 0://frame header must be 0x59
//        Serial.print("Byte0:");
//        Serial.println(c,HEX);
        if(c == 0x59){
          bytenum += 1;
        }
        break;
      case 1://frame header must be 0x59
//        Serial.print("Byte1:");
//        Serial.println(c,HEX);
        if(c == 0x59){
          bytenum += 1;
        }
        break;
      case 2://distance value low 8 bits
//        Serial.print("Byte2:");
//        Serial.println(c,HEX);
        distance = c;
        bytenum += 1;
        break;
      case 3://distance value high 8 bits
//        Serial.print("Byte3:");
//        Serial.println(c,HEX);
        distance = distance + 256*(int)c;
        if(0<distance && distance < 1000){
          LIDAR_buf = distance;
        }else{
          distance = LIDAR_buf;
        }
        Serial.print(":deg_PITCH");
        Serial.print(deg_PITCH);
        Serial.print(":deg_YAW");
        Serial.print(deg_YAW);
        Serial.print(":distance:");
        Serial.println(distance);
        distance = 0;
        bytenum += 1;
        break;
      case 4://strength value low 8 bits
//        Serial.print("Byte4:");
//        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 5://strength value high 8 bits
//        Serial.print("Byte5:");
//        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 6://Temp_L low 8 bits
//        Serial.print("Byte6:");
//        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 7://Temp_H high 8 bits
//        Serial.print("Byte7:");
//        Serial.println(c,HEX);
        bytenum += 1;
        break;
      case 8://checksum
//        Serial.print("Byte8:");
//        Serial.println(c,HEX);
        bytenum = 0;
        break;
    }
  }
}