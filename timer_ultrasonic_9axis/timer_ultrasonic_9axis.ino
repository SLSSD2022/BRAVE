const int ENABLE = 3;
const int CH1 = 4;
const int CH2 = 5;
const int CH3 = 6;
#include<Wire.h>

const int CH4 = 7;
int TRIG = 10;
int ECHO = 9;
long duration, cm;
boolean stop_flag = 0;

double Calib = 110; //キャリブレーション用定数
double Calibx = 11.5;
double Caliby = 65;

int   xMag  = 0;
int   yMag  = 0;
int   zMag  = 0;

#define Addr_Mag 0x13

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); 

  BMX055_Init();

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

  BMX055_Mag();

  float x = atan2(yMag-Caliby,xMag-Calibx)/3.14*180+180;
  x = (x+Calib);

  if (x>360) {
    x = x-360;
  }

  else if (x<0){
   x = x+360;
  }

  else {
   x = x;
 }
Serial.println(x);


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

void BMX055_Init()
{

 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}

//=====================================================================================//
void BMX055_Mag() {  
  unsigned int data[8];
  
  for (int i = 0; i < 8; i++)
  {     
    Wire.beginTransmission(Addr_Mag);
    Wire.write((0x42 + i));    // Select data register
    Wire.endTransmission();
    Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
    // Read 6 bytes of data
    // xMag lsb, xMag msb, yMag lsb
    if (Wire.available() == 1)
      data[i] = Wire.read();
  }
// Convert the data
  xMag = ((data[1] <<5) | (data[0]>>3));
  if (xMag > 4095)  xMag -= 8192;
  yMag = ((data[3] <<5) | (data[2]>>3));
  if (yMag > 4095)  yMag -= 8192;
  zMag = ((data[5] <<7) | (data[4]>>1));
  if (zMag > 16383)  zMag -= 32768;
}
