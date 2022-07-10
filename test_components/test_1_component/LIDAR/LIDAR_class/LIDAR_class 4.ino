#include "./LIDAR.h"
#include <HardwareSerial.h>

LIDAR lidar(&Serial3);
boolean read_flag = 0;
int t = 0;
int bytenum = 0;
int distance = 0;
  
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  lidar.init();
}

void loop() { 
//  while (HWSerial->available() > 0) {
//    byte c = HWSerial->read();
////    Serial.println(c,HEX);
//    switch(bytenum){
//      case 0://frame header must be 0x59
////        Serial.print("Byte0:");
////        Serial.println(c,HEX);
//        if(c == 0x59){
//          bytenum += 1;
//        }
//        break;
//      case 1://frame header must be 0x59
////        Serial.print("Byte1:");
////        Serial.println(c,HEX);
//        if(c == 0x59){
//          bytenum += 1;
//        }
//        break;
//      case 2://distance value low 8 bits
////        Serial.print("Byte2:");
////        Serial.println(c,HEX);
//        distance = c;
//        bytenum += 1;
//        break;
//      case 3://distance value high 8 bits
////        Serial.print("Byte3:");
////        Serial.println(c,HEX);
//        distance = distance + 256*(int)c;
//        Serial.print("distance:");
//        Serial.println(distance);
//        distance = 0;
//        bytenum += 1;
//        break;
//      case 4://strength value low 8 bits
////        Serial.print("Byte4:");
////        Serial.println(c,HEX);
//        bytenum += 1;
//        break;
//      case 5://strength value high 8 bits
////        Serial.print("Byte5:");
////        Serial.println(c,HEX);
//        bytenum += 1;
//        break;
//      case 6://Temp_L low 8 bits
////        Serial.print("Byte6:");
////        Serial.println(c,HEX);
//        bytenum += 1;
//        break;
//      case 7://Temp_H high 8 bits
////        Serial.print("Byte7:");
////        Serial.println(c,HEX);
//        bytenum += 1;
//        break;
//      case 8://checksum
////        Serial.print("Byte8:");
////        Serial.println(c,HEX);
//        bytenum = 0;
//        break;
//    }
//  }
  Serial.println(lidar.getDistance());
  Serial.println("-------------loop-------------");
}
