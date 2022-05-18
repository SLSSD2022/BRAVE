                                      #include <TinyGPSPlus.h>
#include <TinyGPS++.h>


#include <SoftwareSerial.h>
 
// rxPin = 2  txPin = 3
SoftwareSerial mySerial(2, 3);
 
void setup() {
  mySerial.begin(9600);
  Serial.begin(115200);
}
void loop() {
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
}
