const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
const int CH4 = 7;

void setup() {
  // put your setup fcode here, to run once:
  Serial.begin(9600);
  pinMode(ENABLE,OUTPUT);  // 7番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 10番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 11版ピンをOUTPUT指定
  pinMode(CH3,OUTPUT);    // 12番ピンをOUTPUT指定
  pinMode(CH4,OUTPUT);    // 13版ピンをOUTPUT指定

  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable
  delay(500);
}

void loop() {
    Serial.println("1:Normal rotaion");
    digitalWrite(ENABLE,HIGH); // enable on
//    digitalWrite(CH1,HIGH);
    analogWrite(CH1,0.4*255);    
    digitalWrite(CH2,LOW); 
    analogWrite(CH3,0.4*255);    
    digitalWrite(CH4,LOW); 
    delay(4000);
////
//    Serial.println("0:STOP");
//    digitalWrite(ENABLE,LOW); // disable
//    delay(1000);
//    
//    Serial.println("2:Reverse rotation");
//    digitalWrite(ENABLE,LOW); // enable on
//    delay(100);                // 低速で回転させるための調整時間
//    digitalWrite(ENABLE,HIGH); // enable on
//    analogWrite(CH2,1*255);    
//    digitalWrite(CH1,LOW); 
//    digitalWrite(CH4,1*255);    
//    digitalWrite(CH3,LOW); 
//    delay(4000);
    
//    Serial.println("0:STOP");
//    digitalWrite(ENABLE,LOW); // disable
//    delay(1000);

//    Serial.println("3:Twin rotation");
//    digitalWrite(ENABLE,HIGH); // enable on
//    analogWrite(CH1,0.5*255);    
//    digitalWrite(CH2,LOW); 
//    analogWrite(CH4,0.5*255);    
//    digitalWrite(CH3,LOW); 
//    delay(4000);
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    digitalWrite(CH1,HIGH);
    digitalWrite(CH2,HIGH); 
    digitalWrite(CH3,HIGH);    
    digitalWrite(CH4,HIGH); 
    delay(1000);
}
