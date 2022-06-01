#include <TinyGPSPlus.h>

TinyGPSPlus gps;
//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

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
//      Serial.println("YEAHHHHHHHHHHHHHHHHHHHHH!");
//       Serial.print("LAT=");
      Serial.print(gps.location.lat(), 13);
//       Serial.print("LONG=");
      Serial.print(",");
      Serial.println(gps.location.lng(), 13);
    }
  }
}
