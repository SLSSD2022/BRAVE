#ifndef _ULTRASONIC_SENSOR_HEAD_
#define _ULTRASONIC_SENSOR_HEAD_

//------------------------------Ultrasonic sensor------------------------------

class Ultrasonic{
  private:
    //Ultrasonic sensor(short)Bottom
    const uint8_t trigPin;
    const uint8_t echoPin;
    const uint8_t readPin;
    const uint8_t mode;
  public:
    unsigned int distance;

    Ultrasonic(const uint8_t read);
    Ultrasonic(const uint8_t trig,const uint8_t echo);
    void getDistance();
};

#endif
