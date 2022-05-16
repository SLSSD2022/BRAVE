                                      #include <TinyGPSPlus.h>
#include <TinyGPS++.h>


#include <SoftwareSerial.h>
 
// rxPin = 13  txPin = 12
SoftwareSerial mySerial(13, 12);
 
void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
}
void loop() {
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
