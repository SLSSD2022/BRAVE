const int TrigPIN =12; 
const int EchoPIN = 13;
const int REDPIN = 7;
const int YELPIN = 6;
const int BLUPIN = 5;
#define READPIN A0

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(REDPIN,OUTPUT);
  pinMode(YELPIN,OUTPUT);
  pinMode(BLUPIN,OUTPUT);
  pinMode(READPIN,INPUT_PULLUP);
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
    digitalWrite(REDPIN,HIGH);
    digitalWrite(YELPIN,LOW);
    digitalWrite(BLUPIN,LOW);
  }else if(10 <= cm && cm < 20){
    digitalWrite(REDPIN,LOW);
    digitalWrite(YELPIN,HIGH);
    digitalWrite(BLUPIN,LOW);
  }else{
    digitalWrite(REDPIN,LOW);
    digitalWrite(YELPIN,LOW);
    digitalWrite(BLUPIN,HIGH);
  }
  int check = analogRead(READPIN);
  Serial.print(cm);
  Serial.print(":");
  Serial.println(check);
  delay(100);
}

long microsecTocm(long microsec){
  return microsec /29 /2;
}
