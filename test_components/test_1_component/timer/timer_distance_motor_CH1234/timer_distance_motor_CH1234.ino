const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
const int CH4 = 7;
int TRIG = 10;
int ECHO = 9;
long duration, cm;
boolean stop_flag = 0;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); 
  pinMode(ENABLE,OUTPUT);  // 3番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 4番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 5版ピンをOUTPUT指定
  pinMode(CH3,OUTPUT);    // 6番ピンをOUTPUT指定
  pinMode(CH4,OUTPUT);    // 7版ピンをOUTPUT指定
  pinMode(ECHO,INPUT);
  pinMode(TRIG,OUTPUT);
  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable

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
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH1,HIGH);    
    digitalWrite(CH2,HIGH); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,HIGH); 
    digitalWrite(ENABLE,LOW); // disable
  }else{
    Serial.println("GO!GO!");
    stop_flag = 0;
  }
}

void loop() {
    digitalWrite(ENABLE,LOW); // disable
    if(stop_flag == 0){ 
      Serial.println("1:Normal rotaion");
      digitalWrite(ENABLE,HIGH); // enable on
      digitalWrite(CH1,HIGH);    
      digitalWrite(CH2,LOW); 
      digitalWrite(CH3,HIGH);    
      digitalWrite(CH4,LOW);
      delay(1000); 
    }

    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(400);
    
    if(stop_flag == 0){ 
      Serial.println("2:Reverse rotation");
      digitalWrite(ENABLE,HIGH); // enable on
      analogWrite(CH1,LOW);
      digitalWrite(CH2,HIGH); 
      analogWrite(CH3,LOW);
      digitalWrite(CH4,HIGH); 
      delay(1000);
    }
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(400);

    if(stop_flag == 0){ 
      Serial.println("2:Reverse rotation");
      digitalWrite(ENABLE,HIGH); // enable on
      analogWrite(CH1,LOW);
      digitalWrite(CH2,HIGH); 
      analogWrite(CH3,LOW);
      digitalWrite(CH4,HIGH);
      delay(1000); 
    }
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(400);
}


long microsecTocm(long microsec){
  return microsec /29 /2;
}
