const int ENABLE = 7;

const int CH3 = 9;
const int CH4 = 8;
const int CH1 = 11;
const int CH2 = 10;
const int TrigPIN =12; 
const int EchoPIN = 13;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENABLE,OUTPUT);  // 7番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 5番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 3版ピンをOUTPUT指定
  pinMode(CH3,OUTPUT);    // 5番ピンをOUTPUT指定
  pinMode(CH4,OUTPUT);    // 3版ピンをOUTPUT指定

  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable
  delay(500);
}


void loop() {
  long duration, cm;
  pinMode(TrigPIN,OUTPUT);
  digitalWrite(TrigPIN,LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPIN,HIGH);
  delayMicroseconds(10);
  pinMode(EchoPIN,INPUT);
  duration = pulseIn(EchoPIN,HIGH);
  cm = microsecTocm(duration);
  Serial.println(cm);
  if(cm < 10){
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    
  }else if(10 <= cm && cm < 20){
    
    Serial.println("1:Normal rotaion");
    digitalWrite(ENABLE,HIGH); // enable on   
    analogWrite(CH1,127);    
    digitalWrite(CH2,LOW);   
    analogWrite(CH3,127);    
    digitalWrite(CH4,LOW); 
    delay(1000);
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
    
    Serial.println("2:Reverse rotation");
    digitalWrite(ENABLE,HIGH); // enable on
    analogWrite(CH2,127);
    digitalWrite(CH1,LOW);
    analogWrite(CH4,127);
    digitalWrite(CH3,LOW);
    delay(1000);
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
  }else{ 
    
    Serial.println("1:Normal rotaion");
    digitalWrite(ENABLE,HIGH); // enable on
    
    digitalWrite(CH1,HIGH);    
    digitalWrite(CH2,LOW); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,LOW); 
    delay(1000);
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
    
    
    Serial.println("2:Reverse rotation");
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH2,HIGH);
    digitalWrite(CH1,LOW);
    digitalWrite(CH4,HIGH);
    digitalWrite(CH3,LOW);
    delay(1000);
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(1000);
  }
  Serial.print(cm);
  Serial.print(":");
  delay(100);
}

long microsecTocm(long microsec){
  return microsec /29 /2;
}
