#include "./Motor.h"

Motor::Motor(uint8_t enable,uint8_t ch1,uint8_t ch2,uint8_t ch3,uint8_t ch4)
    :ENABLE(enable)
    ,CH1(ch1)
    ,CH2(ch2)
    ,CH3(ch3)
    ,CH4(ch4)
    ,controlStatus(0)
    ,speedL(0)
    ,speedR(0)
    ,threshold(10) //角度の差分の閾値
    ,spinThreshold(10) //角度の差分の閾値
    ,deltaTheta(0)//目的方向と姿勢の相対角度差
{
}

void Motor::init()
{
    pinMode(ENABLE, OUTPUT);
    pinMode(CH1, OUTPUT);
    pinMode(CH2, OUTPUT);
    pinMode(CH3, OUTPUT);
    pinMode(CH4, OUTPUT);
    digitalWrite(ENABLE, LOW); // disable
    Serial.println("Motor driver initailized!");
    return;
}

void Motor::setThreshold(int threshold,int spinthreshold)
{
    this->threshold = threshold;
    this->spinThreshold = spinthreshold;
    return;
}


void Motor::stop()
{
    digitalWrite(ENABLE, HIGH);
    digitalWrite(CH1, HIGH);
    digitalWrite(CH2, HIGH);
    digitalWrite(CH3, HIGH);
    digitalWrite(CH4, HIGH);
    this->controlStatus = 0;//"stop"
    this->speedR = 0;
    this->speedL = 0;
    Serial.print(":stop!");
    return;
}


void Motor::right(int speed)
{
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, speed);
    digitalWrite(CH2, LOW);
    this->speedR = speed;
    this->speedL = 0;
    Serial.print(":right");
    return;
}

void Motor::left(int speed)
{
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH3, speed);
    digitalWrite(CH4, LOW);
    this->speedR = 0;
    this->speedL = speed;
    Serial.print(":left");
    return;
}

void Motor::goStraight(int speed)
{
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, speed);
    digitalWrite(CH2, LOW);
    analogWrite(CH3, speed);
    digitalWrite(CH4, LOW);
    this->controlStatus = 1;//"Go straight"
    this->speedR = speed;
    this->speedL = speed;
    Serial.print(":Go straight");
    return;
}

void Motor::turn(int speedl,int speedr)
{
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, speedR);
    digitalWrite(CH2, LOW);
    analogWrite(CH3, speedL);
    digitalWrite(CH4, LOW);
    this->speedL = speedl;
    this->speedR = speedr;
    if(speedl > speedr){
        this->controlStatus = 3;//"turn right"
        Serial.print(":turn right");
        return;
    }
    else
    {
        this->controlStatus = 2;//"turn left"
        Serial.print(":turn left");
        return;
    }
}

void Motor::spinLeft(int speed)
{
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, speed);
    analogWrite(CH2, 0);
    analogWrite(CH3, 0);
    analogWrite(CH4, speed);
    this->controlStatus = 5;//"spin to left"
    this->speedR = speed;
    this->speedL = -speed;
    Serial.print(":spin to left!");
    return;
}

void Motor::spinRight(int speed)
{
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH1, 0);
    analogWrite(CH2, speed);
    analogWrite(CH3, speed);
    analogWrite(CH4, 0);
    this->controlStatus = 4;//"spin to left"
    this->speedR = speed;
    this->speedL = -speed;
    Serial.print(":spin to left!");
    return;
}

void Motor::angleGo(float bodyDeg,float goalDeg,int speed)
{
  int deltaTheta = 0;
  if (bodyDeg < goalDeg) {
    deltaTheta = goalDeg - bodyDeg;
    Serial.print(":bodyDeg < goalDeg:");
    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= this->threshold / 2) || (360 - this->threshold / 2 <= deltaTheta && deltaTheta <= goalDeg)) {
      motor.goStraight(speed);
    }
    //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else if (this->threshold / 2 < deltaTheta && deltaTheta <= 180) {
      motor.turn(speed,speed - (deltaTheta * speed / 180));//"turn right"
    }

    //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else {
      motor.turn(speed - ((360 - deltaTheta) * speed / 180),speed);//"turn left"
    }
  }
  else {
    deltaTheta = bodyDeg - goalDeg;
    Serial.print(":goalDeg < bodyDeg:");
    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= this->threshold / 2) || (360 - this->threshold / 2 <= deltaTheta && deltaTheta <= 360)) {
      motor.goStraight(speed);
    }
    //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else if (this->threshold / 2 < deltaTheta && deltaTheta <= 180) {
      motor.turn(speed - (deltaTheta * speed / 180),speed);//"turn left"
    }

    //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else {
      motor.turn(speed,speed - ((360 - deltaTheta) * speed / 180));//"turn right"

    }
  }
  Serial.print(":speedL:");
  Serial.print(this->speedL);
  Serial.print(":speedR:");
  Serial.print(this->speedR);
  return;
}
