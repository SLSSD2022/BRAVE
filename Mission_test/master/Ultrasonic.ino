#include "./Ultrasonic.h"

Ultrasonic::Ultrasonic(const uint8_t trig,const uint8_t echo)
  :trigPin(trig)
  ,echoPin(echo)
  ,readPin(0)
  ,mode(2)
  ,distance(0)
{
}

Ultrasonic::Ultrasonic(const uint8_t read)
  :trigPin(0)
  ,echoPin(0)
  ,readPin(read)
  ,mode(1)
  ,distance(0)
{
}

void Ultrasonic::getDistance() {
  if(mode == 1)
  {
    unsigned int anVolt = analogRead(readPin);
    this->distance = anVolt / 2;
    return;
  }
  else if(mode == 2)
  {
    unsigned long duration;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    duration = pulseIn(echoPin, HIGH);
    this->distance = (unsigned int) duration / 29 / 2;
    return;
  }
}

void Ultrasonic::init() {
    if(mode == 1)
    {
      //do nothing
      return;
    }
    else if(mode == 2)
    {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      return;
    }
}
  
