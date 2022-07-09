#include "./Ultrasonic.h"

Ultrasonic::Ultrasonic(const uint8_t trig,const uint8_t echo)
  :trigPin(trig)
  ,echoPin(echo)
  ,readPin(0)
  ,mode(2)
  ,distance(0)
{
  //超音波センサ
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
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
  if(this->mode == 1)
  {
    unsigned int anVolt = analogRead(this->readPin);
    this->distance = anVolt / 2;
    return;
  }
  else if(this->mode == 2)
  {
    unsigned long duration;
    digitalWrite(this->trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->trigPin, HIGH);
    delayMicroseconds(10);
    duration = pulseIn(this->echoPin, HIGH);
    this->distance = (unsigned int) duration / 29 / 2;
    return;
  }
}
