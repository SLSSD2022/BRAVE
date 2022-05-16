int TRIG = 10;
int ECHO = 9;
long duration, cm;
boolean stop_flag = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); 
  pinMode(ECHO,INPUT);
  pinMode(TRIG,OUTPUT);

  TCCR1A  = 0;           
  TCCR1B  = 0;          
  TCCR1B |= (1 << WGM12) | (1 << CS12);   //CTC mode //256分周(16micros,62.5kHz)
  OCR1A   = 1000*5-1;            //5000カウント毎(80ms周期,12.5Hz) 31250カウント毎(=0.5s周期)
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(13, !digitalRead(13));
//  Serial.println("Hello world!");
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  duration = pulseIn(ECHO,HIGH);
  cm = microsecTocm(duration);
  if(cm < 10){
    stop_flag = 1;
    Serial.print("STOP!:");
    Serial.println(cm);
  }else{
    Serial.println("GO!GO!");
    stop_flag = 0;
  }
}

void loop() {
}


long microsecTocm(long microsec){
  return microsec /29 /2;
}
