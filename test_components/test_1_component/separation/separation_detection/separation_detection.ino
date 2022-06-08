const int DETECTION_PIN = 4;

void setup(){
  Serial.begin(9600);
  pinMode(DETECTION_PIN,INPUT_PULLUP);
}

void loop(){
  if(digitalRead(DETECTION_PIN) == 1)
  {
    Serial.println("separated!");
  }
  else{
    Serial.println("connected!");
  }
}
