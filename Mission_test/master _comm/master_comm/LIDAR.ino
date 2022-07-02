//=========LIDAR sensor function============================================================================//
unsigned int getLIDAR(unsigned int distanceByLIDAR) {
  /*---------------------------------------//
  //    Data Format and Code Explanation   //
  /-----------------------------------------/
  // Byte0: 0x59
  // Byte1: 0x59
  // Byte2: Dist_L
  // Byte3: Dist_H
  // Byte4: Strength_L
  // Byte5: Strength_H
  // Byte6: Temp_L
  // Byte7: Temp_H
  // Byte8: Checksum 
  //---------------------------------------*/
  unsigned int distByLidarBuf;
  int bytenum = 0;
  while (Serial2.available() > 0 && bytenum < 4)//near_flagは一時的なもの
  {
    byte c = Serial2.read();
    switch (bytenum) {
      case 0://frame header must be 0x59
      case 1://frame header must be 0x59
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
          distByLidarBuf = (int)c;
          bytenum += 1;
        }
        break;
      case 3://distance value high 8 bits
        //        Serial.print("Byte3:");
        //        Serial.println(c,HEX);
        distByLidarBuf += 256 * (int)c;
        //        Serial.print("distance:");
        //        Serial.println(cm_LIDAR);
        bytenum += 1;
        if (0 < distByLidarBuf && distByLidarBuf < 1000) {
          distanceByLIDAR = distByLidarBuf;
          return distByLidarBuf;
        } else {
          return distanceByLIDAR;
        }
        break;
      // case 4://strength value low 8 bits
      // case 5://strength value high 8 bits
      // case 6://Temp_L low 8 bits
      // case 7://Temp_H high 8 bits
      // case 8://checksum
      //   bytenum += 1;
      //   break;
    }
  }
}