const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ENABLE,OUTPUT);  // 3番ピンをOUTPUT指定
  pinMode(CH1,OUTPUT);    // 4番ピンをOUTPUT指定
  pinMode(CH2,OUTPUT);    // 5版ピンをOUTPUT指定

  // 初期化 DCモータが突然動きださないように
  digitalWrite(ENABLE,LOW); // disable
  delay(500);
}

void loop() {
    Serial.println("1:Normal rotaion");
    digitalWrite(ENABLE,HIGH); // enable on
    digitalWrite(CH1,HIGH);    
    digitalWrite(CH2,LOW); 
    delay(1000);

    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(100);
    
    Serial.println("2:Reverse rotation");
    digitalWrite(ENABLE,HIGH); // enable on
    analogWrite(CH1,LOW);
    digitalWrite(CH2,HIGH); 
    delay(1000);
    
    Serial.println("0:STOP");
    digitalWrite(ENABLE,LOW); // disable
    delay(100);
}
