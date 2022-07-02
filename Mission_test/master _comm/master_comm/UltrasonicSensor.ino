#include "./UltrasonicSensor.h"

void UltrasonicSensor::init()
{
  //超音波センサ
  pinMode(HEAD_Trig, OUTPUT);
  pinMode(HEAD_Echo, INPUT);
}

unsigned int UltrasonicSensor::getDistance() {
  unsigned long duration;
  digitalWrite(HEAD_Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(HEAD_Trig, HIGH);
  delayMicroseconds(10);
  duration = pulseIn(HEAD_Echo, HIGH);
  return (unsigned int) duration / 29 / 2;
}
