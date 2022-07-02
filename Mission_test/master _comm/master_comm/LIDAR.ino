#include "./LIDAR.h"

//=========LIDAR sensor function============================================================================//
unsigned int getLIDAR() {
  boolean flagForLIDAR = 1;
  int distanceCalByLIDAR;
  int bytenum = 0;
  while (Serial2.available() > 0 && flagForLIDAR == 1)//near_flagは一時的なもの
  {
    byte c = Serial2.read();
    switch (bytenum) {
      case 0://frame header must be 0x59
        //        Serial.print("Byte0:");
        //        Serial.println(c,HEX);
        if (c == 0x59) {
          bytenum += 1;
        }
        break;
      case 1://frame header must be 0x59
        //        Serial.print("Byte1:");
        //        Serial.println(c,HEX);
        if (c == 0x59) {
          bytenum += 1;
        }
        break;
      case 2://distance value low 8 bits
        //        Serial.print("Byte2:");
        //        Serial.println(c,HEX);
        if (c == 0x59) {
          //多分次がcase2
        }
        else {
          distanceCalByLIDAR = (int)c;
          bytenum += 1;
        }
        break;
      case 3://distance value high 8 bits
        //        Serial.print("Byte3:");
        //        Serial.println(c,HEX);
        distanceCalByLIDAR += 256 * (int)c;
        //        Serial.print("distance:");
        //        Serial.println(cm_LIDAR);
        flagForLIDAR = 0;
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
  flagForLIDAR = 1;
  if (0 < distanceCalByLIDAR && distanceCalByLIDAR < 1000) {
    LIDAR_buf = distanceCalByLIDAR;
    return distanceCalByLIDAR;
  } else {
    return LIDAR_buf;
  }
}