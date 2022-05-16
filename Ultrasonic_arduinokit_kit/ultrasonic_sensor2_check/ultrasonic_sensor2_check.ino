const int TrigPIN =10; 
const int EchoPIN = 9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
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
  Serial.print(cm);
  Serial.print(":");
  delay(100);
}

long microsecTocm(long microsec){
  return microsec /29 /2;
}
