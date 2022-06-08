
const int HEAD_Trig = 22; 
const int HEAD_Echo = 24;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(500);
}


void loop() {
  long duration, cm;
  pinMode(HEAD_Trig,OUTPUT);
  digitalWrite(HEAD_Trig,LOW);
  delayMicroseconds(2);
  digitalWrite(HEAD_Trig,HIGH);
  delayMicroseconds(10);
  pinMode(HEAD_Echo,INPUT);
  duration = pulseIn(HEAD_Echo,HIGH);
  cm = microsecTocm(duration);
  Serial.print(cm);
  Serial.println("");
  delay(100);
}

long microsecTocm(long microsec){
  return microsec /29 /2;
}
