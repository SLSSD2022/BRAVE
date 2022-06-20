/**********************************************
  Arduino nano SPI(Master)
**********************************************/
#include <SPI.h>
 
void setup() {
  Serial.begin(9600);
  Serial.println("/-----START Arduino nano-----/");
  Serial.println("/---------SPI Master--------/");
 
  SPI.begin();
  digitalWrite(SS, HIGH);
}
 
int knob = 54;
void loop() {
  byte rxdata;
   
  knob = 200;
  digitalWrite(SS, LOW);
  SPI.transfer(knob);
  digitalWrite(SS, HIGH);
  digitalWrite(SS, LOW);
  rxdata = SPI.transfer(0);
  digitalWrite(SS, HIGH);
 
  Serial.print("tx:");
  Serial.print(knob);
  Serial.print(" rx:");
  Serial.println(rxdata);
 
  knob = 32;
  digitalWrite(SS, LOW);
  SPI.transfer(knob);
  digitalWrite(SS, HIGH);
  digitalWrite(SS, LOW);
  rxdata = SPI.transfer(0);
  digitalWrite(SS, HIGH);
 
  Serial.print("tx:");
  Serial.print(knob);
  Serial.print(" rx:");
  Serial.println(rxdata);
   
  delay(2000);
 
}
