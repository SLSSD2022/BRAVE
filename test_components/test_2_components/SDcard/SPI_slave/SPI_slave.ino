/**********************************************
  Arduino uno SPI(sleave)
**********************************************/
#include <SPI.h>

byte rxdata;

void setup() {
  Serial.begin(9600);
  Serial.println("/-----START Arduino uno-----/");
  Serial.println("/----------SPI Slave---------/");
 
  SPCR |= bit(SPE);
  pinMode(MISO, OUTPUT);
  SPI.attachInterrupt();
}

void loop() {
}

ISR(SPI_STC_vect) {
  rxdata = SPDR;
  SPDR = rxdata;
}
