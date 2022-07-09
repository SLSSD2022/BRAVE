int DETECTION_PIN = 2;

//モーター
const int ENABLE = 3;
const int CH1 = 5;
const int CH2 = 4;
const int CH3 = 6;
const int CH4 = 7;

void setup(){
  Serial.begin(9600);
  pinMode(2,INPUT_PULLUP);
  
  //モーター
  pinMode(CH1, OUTPUT);
  pinMode(CH2, OUTPUT);
  pinMode(CH3, OUTPUT);
  pinMode(CH4, OUTPUT);
  digitalWrite(ENABLE, LOW); // disable
}

void loop(){
  if(digitalRead(2) == 1)
  {
    digitalWrite(ENABLE,HIGH);
    digitalWrite(CH1,HIGH);
    digitalWrite(CH2,LOW);
    digitalWrite(CH3,HIGH);
    digitalWrite(CH4,LOW);
    Serial.println("separated!");
  }
  else if(digitalRead(2) == 0 ){
    digitalWrite(ENABLE,LOW);
    digitalWrite(CH1,HIGH);
    digitalWrite(CH2,HIGH);
    digitalWrite(CH3,HIGH);
    digitalWrite(CH4,HIGH);
    Serial.println("connected!");
  }
}
