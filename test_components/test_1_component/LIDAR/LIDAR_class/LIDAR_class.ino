#include "./LIDAR.h"
#include <HardwareSerial.h>

LIDAR lidar(&Serial3);
int distance = 0;
  
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  lidar.init();
}

void loop() { 
  Serial.print(lidar.getDistance());
  delay(1000);
}
