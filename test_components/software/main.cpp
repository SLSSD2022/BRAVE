#include <iostream>
using namespace std;


struct Kitty {
	explicit Kitty(char *str) { this->str = str; }
	char * getStr() { return str; }
private:
	char *str;
};


typedef struct _bmx055 {
    float xGyro;
    float yGyro;
    float zGyro;
    int xMag;
    int yMag;
    int zMag;

    //Constant for calibration(最初はセンサに書いてある矢印に対して微妙に0°がずれてるので、ローバーの進行方向と並行な向きの矢印が磁北（0°）になるよう調整）
    float calib;
    float calibx;
    float caliby;
    //char *str;

    _bmx055() ;
    _bmx055(float xG,float yG,float zG,int xM,int yM,int zM,float ca,float cax,float cay) ;
	//char * getStr() { return str; }
} bmx055;

bmx055::_bmx055()
  :xGyro(0)
  ,yGyro(0)
  ,zGyro(0)
  ,xMag(0)
  ,yMag(0)
  ,zMag(0)
  ,calib(175)
  ,calibx(20)
  ,caliby(132)
{
}

bmx055::_bmx055(float xG,float yG,float zG,int xM,int yM,int zM,float ca,float cax,float cay)
  :xGyro(xG)
  ,yGyro(yG)
  ,zGyro(zG)
  ,xMag(xM)
  ,yMag(yM)
  ,zMag(zM)
  ,calib(ca)
  ,calibx(cax)
  ,caliby(cay)
{
}


class UltrasonicSensor{
public:
    //Ultrasonic sensor(short)Bottom
    int trigPin;
    int echoPin;
    int readPin;
    int mode;
    unsigned int distance;
    
    UltrasonicSensor();
    UltrasonicSensor(int readpin);
    UltrasonicSensor(const int trig,const int echo);
    void getDistance();
};

UltrasonicSensor::UltrasonicSensor()
 :trigPin(22)
 ,echoPin(24)
 ,readPin(0)
 ,mode(2)
 ,distance(0)
{
 //超音波センサ
 //pinMode(this->trigPin, OUTPUT);
 //pinMode(this->echoPin, INPUT);
}

// UltrasonicSensor::UltrasonicSensor(int readpin)
//  :trigPin(0)
//  ,echoPin(0)
//  ,readPin(readpin)
//  ,mode(1)
//  ,distance(0)
// {
// }

// UltrasonicSensor::UltrasonicSensor(int trig,int echo)
//  :trigPin(trig)
//  ,echoPin(echo)
//  ,readPin(0)
//  ,mode(2)
//  ,distance(0)
// {
//  //超音波センサ
//  //pinMode(this->trigPin, OUTPUT);
//  //pinMode(this->echoPin, INPUT);
// }



void UltrasonicSensor::getDistance() {
  if(this->mode == 1)
  {
    // unsigned int anVolt = analogRead(this->readPin);
    // this->distance = anVolt / 2;
    return;
  }
  else if(this->mode == 2)
  {
    // unsigned long duration;
    // digitalWrite(this->trigPin, LOW);
    // delayMicroseconds(2);
    // digitalWrite(this->trigPin, HIGH);
    // delayMicroseconds(10);
    // duration = pulseIn(this->echoPin, HIGH);
    // this->distance = (unsigned int) duration / 29 / 2;
    return;
  }
}


int main() {
	bmx055 obj;
	bmx055 obj2(0.00, 0.00, 0.00, 0, 0, 0, 175, 20, 132);

	return 0;
}
