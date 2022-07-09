#include "./Communication.h"
#include "./IMU.h"
#include "./Rover.h"

IMU imu;
Rover rover;
Communication comm(&Serial2,2,3);

void setup() {
  Serial.begin(115200);
  Serial.print("Twelite Rover");
  Serial.println();
  comm.init();
}


void loop() {
  //---------------Initial Mode------------------
  bool Initial = true
  while (Initial) {
    start = millis();
    if (start > (stopi + 1000)) {
      Serial.println("initial Mode: waiting for GPS...");
      comm.sendStatus("waitGPS");//"Separation Detection" in Comms Headers is 1 
      stopi = millis();
    }
    if(comm.receiveGPS()){
      Initial = false;
    }
  }
  



  
  start = millis();
  int timer = start - stopi;

  Serial.print(":start");
  Serial.print(start);
  Serial.print(":stopi");
  Serial.print(stopi);
  Serial.print(":timer");
  Serial.println(timer);

  if (timer > 10000) {
    Serial.println(":Communication start!");
    comm.HKtoGS();
    stopi = millis();
    Serial.println(":Communication end!");
  }
}
