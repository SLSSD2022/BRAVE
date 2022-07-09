const int DETECTION_PIN = 2;

void setup(){
  Serial.begin(9600);
  pinMode(DETECTION_PIN,INPUT_PULLUP);
}

void loop(){
  if(digitalRead(DETECTION_PIN) == 0)
  {
    Serial.println("separated!");
  }
  else{
    Serial.println("connected!");
  }
}
