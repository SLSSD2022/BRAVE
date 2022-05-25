#include <TinyGPS++.h>
#include <SoftwareSerial.h>
 
// rxPin = 13  txPin = 13
SoftwareSerial mySerial(12, 13);
 
void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
}
void loop() {
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
