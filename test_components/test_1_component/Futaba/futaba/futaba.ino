#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13); // RX, TX
int sbusIndex=0;
uint8_t sbus[25] = {0};
unsigned int rcChannel[18];
int failSafe = 0;

void setup(){
 
  Serial.begin(115200); // ハードウェアシリアルを準備
  while (!Serial) {
    ; // シリアルポートの準備ができるのを待つ(Leonardoのみ必要)
  }
  Serial.println("Ready");
  mySerial.begin(100000); // ソフトウェアシリアルの初期化
}
 
void loop(){
//  if (mySerial.available()>0){
//
//    int val = mySerial.read();
    Serial.print(sbusIndex);
//
//
//    if (sbusIndex == 0 && val != 0x0F) {
//        Serial.println("failsafe");
//        failSafe = true;
//        return;
//    }
//
//    sbus[sbusIndex] = val;
    sbusIndex++;


//    if (sbusIndex == 25) {
//        for(int i = 0;i<25;i++){
//              Serial.print(sbus[i]);
//        }
//        Serial.println("");
//        sbusIndex = 0;
//
//        if (sbus[24] != 0x0) {
//            rcChannel[0] = ((sbus[1] | sbus[2] << 8) & 0x07FF);
//            rcChannel[1] = ((sbus[2] >> 3 | sbus[3] << 5) & 0x07FF);
//            rcChannel[2] = ((sbus[3] >> 6 | sbus[4] << 2 | sbus[5] << 10) & 0x07FF);
//            rcChannel[3] = ((sbus[5] >> 1 | sbus[6] << 7) & 0x07FF);
//            rcChannel[4] = ((sbus[6] >> 4 | sbus[7] << 4) & 0x07FF);
//            rcChannel[5] = ((sbus[7] >> 7 | sbus[8] << 1 | sbus[9] << 9) & 0x07FF);
//            rcChannel[6] = ((sbus[9] >> 2 | sbus[10] << 6) & 0x07FF);
//            rcChannel[7] = ((sbus[10] >> 5 | sbus[11] << 3) & 0x07FF);
//            rcChannel[8] = ((sbus[12] | sbus[13] << 8) & 0x07FF);
//            rcChannel[9] = ((sbus[13] >> 3 | sbus[14] << 5) & 0x07FF);
//            rcChannel[10] = ((sbus[14] >> 6 | sbus[15] << 2 | sbus[16] << 10) & 0x07FF);
//            rcChannel[11] = ((sbus[16] >> 1 | sbus[17] << 7) & 0x07FF);
//            rcChannel[12] = ((sbus[17] >> 4 | sbus[18] << 4) & 0x07FF);
//            rcChannel[13] = ((sbus[18] >> 7 | sbus[19] << 1 | sbus[20] << 9) & 0x07FF);
//            rcChannel[14] = ((sbus[20] >> 2 | sbus[21] << 6) & 0x07FF);
//            rcChannel[15] = ((sbus[21] >> 5 | sbus[22] << 3) & 0x07FF);
//            for(int i = 0;i<16;i++){
//              Serial.print(rcChannel[i]);
//              Serial.print(",");
//            }
//            Serial.println("");
//
//            if ((sbus[23] >> 3) & 0x0001) {
//                // コントローラがOFF状態
//                failSafe = true;
//            } else {
//                failSafe = false;
//            }
//        }
//    }
//  }
}
