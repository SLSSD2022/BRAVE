#include "./GPS.h"

GPS gps(&Serial1);
float latA = 35.7155075073242;
float lngA = 139.761245727539;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000);
  gps.init();
}

void loop() { // run over and over
  float latR = 0;
  float lngR = 0;
  float x = 0;
  float y = 0;
  gps.updateGPSlocation(&latR,&lngR);
  Serial.print(latR,13);
  Serial.print(",");
  Serial.print(lngR,13);
  float RtoA = gps.distanceBetween(latR, lngR, latA, lngA);
  Serial.print(",");
  Serial.print(RtoA, 2);
  if(gps.S_ == 0){
    gps.calc_const(latA,lngA);
  }
  gps.calc_xy(latA,lngA, &x, &y);
  Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println("");
}
