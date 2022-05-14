#include <SoftwareSerial.h>



// MaxSonar Arduino
//      GND GND
//      +5V VCC
//       TX 2
//       RX 3
//       AN Analog 0
//       PW 4

SoftwareSerial sonar(2, 3, true); // 3番目の引数：論理を反転する
const int anPin = 0;
const int pwPin = 4;

void setup()
{
  Serial.begin(57600);
  sonar.begin(9600);
  pinMode(pwPin, INPUT);
}

void loop()
{
  // アナログポートから読み込む
  int range_an = analogRead(anPin);
  int inch = range_an / 2; // VCC/512 per inch

  Serial.print( inch );
  Serial.print( " inch (" );
  Serial.print( ((unsigned int)(inch) * (unsigned int)(2.54 * 64)) >> 6 );
  Serial.println( " cm) from Analog" );

  // パルスポートから読み込む
  unsigned long range_pw = pulseIn(pwPin, HIGH);
  inch = range_pw / 147; // 147us per inch

  Serial.print( inch );
  Serial.print( " inch (" );
  Serial.print( ((unsigned int)(inch) * (unsigned int)(2.54 * 64)) >> 6 );
  Serial.println( " cm) from Pulse" );

  // シリアルポートから読み込む
  while( sonar.available() >= 4 ){
    // シリアル出力は大文字のRから始まって、
    // 000から255の3桁の距離（インチ）に続き、
    // キャリッジリターン(13)が送られてくる。
    if( sonar.read() == 'R' ){
      Serial.print( (char)sonar.read() );
      Serial.print( (char)sonar.read() );
      Serial.print( (char)sonar.read() );
      Serial.println(" inch from Serial");
      sonar.flush();
    }
  }

  delay(60);
}
