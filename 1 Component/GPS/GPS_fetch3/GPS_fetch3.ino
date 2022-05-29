#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
//SoftwareSerial mySerial(23, 22); // RX, TX
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

  Serial2.begin(9600);
  while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  //mySerial.begin(9600);
  // mySerial.println("Hello, world?");
}

void loop() { // run over and over

  while (Serial1.available() > 0) {
    char c = Serial1.read();
    //Serial.print(c);
    gps.encode(c);
    if (gps.location.isUpdated()) {
      // Serial.print("LAT=");
      Serial.print(gps.location.lat(), 6);
      // Serial.print("LONG=");
      Serial.print(",");
      Serial.println(gps.location.lng(), 6);
    }
  }

  while (Serial2.available() > 0) {
    char cc = Serial2.read();
    //Serial.print(c);
    gps.encode(cc);
    if (gps.location.isUpdated()) {
      // Serial.print("LAT=");
      Serial.print(gps.location.lat(), 6);
      // Serial.print("LONG=");
      Serial.print(",");
      Serial.println(gps.location.lng(), 6);
    }
  }
}
