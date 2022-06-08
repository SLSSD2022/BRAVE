const int DETECTION_PIN = 4;

//モーター
const int ENABLE = 8;
const int CH1 = 9;
const int CH2 = 11;
const int CH3 = 10;
const int CH4 = 12;

void setup(){
  Serial.begin(9600);
  pinMode(DETECTION_PIN,INPUT_PULLUP);
  
  //モーター
  pinMode(CH1, OUTPUT);
  pinMode(CH2, OUTPUT);
  pinMode(CH3, OUTPUT);
  pinMode(CH4, OUTPUT);
  digitalWrite(ENABLE, LOW); // disable
}

void loop(){
  if(digitalRead(DETECTION_PIN) == 1)
  {
    digitalWrite(ENABLE,HIGH);
    digitalWrite(CH1,HIGH);
    digitalWrite(CH2,LOW);
    digitalWrite(CH3,HIGH);
    digitalWrite(CH4,LOW);
    Serial.println("separated!");
  }
  else{
    digitalWrite(ENABLE,LOW);
    digitalWrite(CH1,HIGH);
    digitalWrite(CH2,HIGH);
    digitalWrite(CH3,HIGH);
    digitalWrite(CH4,HIGH);
    Serial.println("connected!");
  }
}
