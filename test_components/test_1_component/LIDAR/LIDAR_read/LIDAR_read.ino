boolean read_flag =0;
int t =0;
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial2.begin(115200);
  while (!Serial2) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() { // run over and over

  while (Serial2.available() > 0) {
    byte c = Serial2.read();
//    Serial.println(c,HEX);
    if(read_flag == 1){
      Serial.println(c);
      read_flag =0;
    }
    if(c == 0x59){
      t += 1;
    }
    if(t == 1){
      read_flag = 1;
      t = 0;
    }
  }
}
