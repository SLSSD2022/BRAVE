#include "./communication.h"
#include "./rover.h"
#include "./GPS.h"

//=========Communication function============================================================================//
void processData() {
  // character data is converted to uint8_t data here
  // and is stored in the encodedRx[] buffer
  int i = 7;
  int d, e;
  int checker;
  while (i < bufferPos - 4) {//What is lentCtr???
    checker = buffRx[i] & 0b01000000;
    //Check if the 2nd MSB is 1 or 0. If 1: it's A-F letter in ASCII
    if (checker == 0b01000000) {
      d = buffRx[i] + 9; //makes the corresponding letter to appear in its HEX representation at the last 4 bits ex:if A is your character in ASCII is 01000001, add 9 -> 01001010 is 0X4A.
    }
    else {
      d = buffRx[i];
    }
    d = d << 4; // Shift the letter to the first 4 MSB ex: 0X4A -> 0XA4
    d = d & 0b11110000; // Remove remaining bits obtaining for ex: 0XA0 if A was your character.
    //Same for the following character
    checker = buffRx[i + 1] & 0b01000000;
    if (checker == 0b01000000) {
      e = buffRx[i + 1] + 9;
    }
    else {
      e = buffRx[i + 1];
    }
    e = e & 0b00001111;
    encodedRx[(i - 7) / 2] = d + e;
    i = i + 2;
  }
}

void encodeCyclic() {
  uint8_t ctr = 0;
  uint8_t m;
  while (ctr < sizeof(roverData)) {
    m = packetTx.packetData[ctr] >> 4;
    encodedTx[2 * ctr] = ((m & 1) * generator[3]) ^ (((m >> 1) & 1) * generator[2]) ^
                         (((m >> 2) & 1) * generator[1]) ^ (((m >> 3) & 1) * generator[0]);
    //Serial.print(encodedTx[2*ctr],HEX);
    m = packetTx.packetData[ctr];
    encodedTx[2 * ctr + 1] = ((m & 1) * generator[3]) ^ (((m >> 1) & 1) * generator[2]) ^
                             (((m >> 2) & 1) * generator[1]) ^ (((m >> 3) & 1) * generator[0]);
    //Serial.println(encodedTx[2*ctr+1],HEX);
    ctr++;
  }
}

bool checkError(uint8_t dataByte) {
  uint8_t p[3];
  uint8_t ctr = 0;
  p[0] = dataByte & parityCheck[0];
  p[1] = dataByte & parityCheck[1];
  p[2] = dataByte & parityCheck[2];
  while (ctr < sizeof(gpsDataStruct) / 2) {
    p[0] = (p[0] & 1) ^ (p[0] >> 1);
    p[1] = (p[1] & 1) ^ (p[1] >> 1);
    p[2] = (p[2] & 1) ^ (p[2] >> 1);
    ctr++;
  }
  return (p[0] > 0) || (p[1] > 0) || (p[2] > 0);
}

boolean decodeCyclic() {
  uint8_t ctr = 0;
  bool error[2];
  while (ctr < sizeof(gpsDataStruct)) {
    encodedRx[2 * ctr] = encodedRx[2 * ctr] & 0x7F;
    encodedRx[2 * ctr + 1] = encodedRx[2 * ctr + 1] & 0x7F;
    error[0] = checkError(encodedRx[2 * ctr]);
    error[1] = checkError(encodedRx[2 * ctr + 1]);
    dataRx.gpsBytes[ctr] = ((encodedRx[2 * ctr] << 1) & 0xF0) +
                           ((encodedRx[2 * ctr + 1] >> 3) & 0x0F); //populate GPS data of goals in dataRx
    if (error[0] || error[1]) { //NACK
      Serial2.print(":000102X\r\n");
      return true;
    }
    ctr++;
  }
  //If no errors send ACK
  Serial2.print(":000101X\r\n");
  return false;
}

void commToGS() {
  unsigned long commStart;
  unsigned long commStop;

  writeToTwelite();//send HK firstly
  Serial.println("Data transmission");

  commStop = millis();
  while (1) { //then go into waiting loop for ACK or NACK
    commStart = millis();
    if (commStart > commStop + 100) { //if 20ms passes, then send HK again
      writeToTwelite();
      Serial.println("timeout:100ms");
      break;
    }
    if (Serial2.available() > 0) {
      char c = Serial2.read();
      if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) {
        buffRx[bufferPos] = c;
        bufferPos++;
      }
      else
      {
        buffRx[bufferPos] = '\0';
        //Checks
        if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '0') { //Arbitrary packet for Rover
          //Serial.println(Buffer);
          if (buffRx[6] == '2') { //NACK
            Serial.print("NACK: Resending packet...");
            writeToTwelite();
          } else if (buffRx[6] == '1') { //ACK
            Serial.print("ACK Received!");
            break;
          }
        }
        //Serial.println(buffRx);
        bufferPos = 0;
      }
    }
  }
  bufferPos = 0;
}

void  writeToTwelite () {
  int ctr1 = 0;
  packetTx.message.setAllData(xMag,yMag,calibx,caliby,x,cm_LIDAR,latR,lngR,degRtoA,controlStatus,overallTime);
  packetTx.message.printAllData();
  encodeCyclic();
  Serial2.print(":000100");
  //Serial.print(":000100");
  while (ctr1 < 2 * sizeof(roverData)) {
    if ((uint8_t)encodedTx[ctr1] < 16) {
      Serial2.print("0");
      //Serial.print("0");
    }
    Serial2.print(encodedTx[ctr1], HEX);
    //Serial.print(encodedTx[ctr1],HEX);
    ctr1++;
  }
  Serial2.print("X\r\n");
  //Serial.print("X\r\n");
}

boolean receiveGPS(){
  if (Serial2.available() > 0) {
    char c = Serial2.read();
    //Serial2.print(c);
    if ( c != '\n' && (bufferPos < MaxBufferSize - 1) ) { //read as data in one packet before it receives "\n"
      buffRx[bufferPos] = c;
      bufferPos++;
      buffRx[bufferPos] = '\0';
    }
    else //Check the buffa if it reads the last character in one packet
    {
      if (buffRx[3] == '0' && buffRx[4] == '1' && buffRx[5] == '0') { //Arbitrary packet for Rover
        if (buffRx[6] == '2') { //NACK
          //do nothing
        }
        else if (buffRx[6] == '1') { //ACK
          //do nothing
        }
        else if (buffRx[6] == '0') { //DATARECEIVE
          processData();//character data is converted to uint8_t data here and is stored in the encodedRx[] buffer
          decodeCyclic();//decode GPS data of three goals


          Serial.println("------------------------initial MODE SUCCESS!!!------------------------");
          Serial.print("1st GPS:");
          Serial.print(dataRx.gpsData.latA[0]);
          Serial.print(",");
          Serial.println(dataRx.gpsData.lngA[0]);
          Serial.print("2nd GPS:");
          Serial.print(dataRx.gpsData.latA[1]);
          Serial.print(",");
          Serial.println(dataRx.gpsData.lngA[1]);
          Serial.print("3rd GPS:");
          Serial.print(dataRx.gpsData.latA[2]);
          Serial.print(",");
          Serial.println(dataRx.gpsData.lngA[2]);
          Serial.println("---------------------------------------------------------------------");


          LogGPSdata();//log the gps data of destination to EEPROM
          Serial2.print(":000101X\r\n"); //Send ACK to MC
          return true;
        }
      }
      //Serial.println(buffRx);
      bufferPos = 0;
    }
  }
  return false;
}
