void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); 

  TCCR1A  = 0;           
  TCCR1B  = 0;          
  TCCR1B |= (1 << WGM12) | (1 << CS12);   //CTC mode //256分周(16micros,62.5kHz)
  OCR1A   = 31250-1;            //5000カウント毎(80ms周期,12.5Hz) 31250カウント毎(=0.5s周期)
  TIMSK1 |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(13, !digitalRead(13));
}

void loop() {
}
