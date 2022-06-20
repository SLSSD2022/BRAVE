#include <TinyGPSPlus.h>

TinyGPSPlus gps;
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

float LatR = 35.719970703125;
float LngR = 139.7361145019531;
float LatA;
float LngA;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000);
  
  Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
}

void loop() { // run over and over
  while (Serial1.available() > 0) {
    char c = Serial1.read();
//    Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated()) {
      LatA = gps.location.lat();
      LngA = gps.location.lng();
      Serial.print(LatA, 13);
      Serial.print(",");
      Serial.print(LngA, 13);
      Serial.print(",");
      Serial.print(gps.distanceBetween(LatR,LngR,LatA,LngA));
      Serial.println("");
  
    }
  }
}


double deg2rad(double deg)
{
  return (double)(deg * PI / 180.0);
}
