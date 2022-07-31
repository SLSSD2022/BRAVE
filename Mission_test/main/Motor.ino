#include "./Motor.h"

Motor::Motor(uint8_t enable,uint8_t ch1,uint8_t ch2,uint8_t ch3,uint8_t ch4)
    :ENABLE(enable)
    ,CH1(ch1)
    ,CH2(ch2)
    ,CH3(ch3)
    ,CH4(ch4)
    ,controlStatus(0)
    ,powerL(0)
    ,powerR(0)
    ,threshold(10) //角度の差分の閾値
    ,spinThreshold(10) //角度の差分の閾値
    ,deltaTheta(0)//目的方向と姿勢の相対角度差
    ,deltaLR(0.95)
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

void Motor::right(int power)
{
    if(power > 0){
      digitalWrite(ENABLE, HIGH); // enable
      analogWrite(CH1, power);
      digitalWrite(CH2, LOW);
    }
    else if(power < 0){
      digitalWrite(ENABLE, HIGH); // enable
      digitalWrite(CH1, LOW);
      analogWrite(CH2, power);
    }
    else{//power == 0
      digitalWrite(ENABLE, HIGH);
      digitalWrite(CH1, HIGH);
      digitalWrite(CH2, HIGH);
    }
    this->powerR = power;
    Serial.print(":powerR:");
    Serial.print(power);
    return;
}

void Motor::left(int power)
{
  int newpower = (int)power*this->deltaLR;
  if(power > 0){
    digitalWrite(ENABLE, HIGH); // enable
    analogWrite(CH3, newpower);
    digitalWrite(CH4, LOW);
  }
  else if(power < 0){
    digitalWrite(ENABLE, HIGH); // enable
    digitalWrite(CH3, LOW);
    analogWrite(CH4, newpower);
  }
  else{//power == 0
    digitalWrite(ENABLE, HIGH);
    digitalWrite(CH3, HIGH);
    digitalWrite(CH4, HIGH);
  }
  this->powerL = newpower;
  Serial.print(":powerL:");
  Serial.print(newpower);
  return;
}


void Motor::stop()
{
    this->right(0);
    this->left(0);
    this->controlStatus = 0;//"stop"
    this->powerR = 0;
    this->powerL = 0;
//    Serial.print(":stop!");
    return;
}


void Motor::goStraight(int power)
{
    this->right(power);
    this->left(power);
    this->controlStatus = 1;//"Go straight"
//    Serial.print(":Go straight");
    return;
}

void Motor::turn(int powerl,int powerr)
{
    this->right(powerr);
    this->left(powerl);
    if(powerl > powerr){
        this->controlStatus = 3;//"turn right"
//        Serial.print(":turn right");
        return;
    }
    else
    {
        this->controlStatus = 2;//"turn left"
//        Serial.print(":turn left");
        return;
    }
}

void Motor::spinRight(int power)
{
    this->right(-power);
    this->left(power);
    this->controlStatus = 4;//"spin to right"
//    Serial.print(":spin to right!");
    return;
}

void Motor::spinLeft(int power)
{
    this->right(power);
    this->left(-power);
    this->controlStatus = 5;//"spin to right"
//    Serial.print(":spin to left!");
    return;
}

void Motor::angleGo(float bodyDeg,float goalDeg,int power)
{
  int deltaTheta = 0;
  if (bodyDeg < goalDeg) {
    deltaTheta = goalDeg - bodyDeg;
//    Serial.print(":bodyDeg < goalDeg:");
//    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= this->threshold / 2) || (360 - this->threshold / 2 <= deltaTheta && deltaTheta <= goalDeg)) {
      motor.goStraight(power);
    }
    //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else if (this->threshold / 2 < deltaTheta && deltaTheta <= 180) {
      motor.turn(power,power - (deltaTheta * power / 180));//"turn right"
    }

    //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else {
      motor.turn(power - ((360 - deltaTheta) * power / 180),power);//"turn left"
    }
  }
  else {
    deltaTheta = bodyDeg - goalDeg;
//    Serial.print(":goalDeg < bodyDeg:");
//    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= this->threshold / 2) || (360 - this->threshold / 2 <= deltaTheta && deltaTheta <= 360)) {
      motor.goStraight(power);
    }
    //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else if (this->threshold / 2 < deltaTheta && deltaTheta <= 180) {
      motor.turn(power - (deltaTheta * power / 180),power);//"turn left"
    }

    //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else {
      motor.turn(power,power - ((360 - deltaTheta) * power / 180));//"turn right"

    }
  }
  return;
}

void Motor::angleSpin(float bodyDeg,float goalDeg)
{
  int deltaTheta = 0;
  if (bodyDeg < goalDeg) {
    deltaTheta = goalDeg - bodyDeg;
//    Serial.print(":bodyDeg < goalDeg:");
//    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= this->threshold / 2) || (360 - this->threshold / 2 <= deltaTheta && deltaTheta <= goalDeg)) {
      motor.stop();
    }
    //閾値よりプラスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else if (this->threshold / 2 < deltaTheta && deltaTheta <= 180) {
      motor.spinRight(deltaTheta * 120 / 180);//"spin right"
    }

    //閾値よりマイナスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else {
      motor.spinLeft(deltaTheta * 120 / 180);//"spin left"
    }
  }
  else {
    deltaTheta = bodyDeg - goalDeg;
//    Serial.print(":goalDeg < bodyDeg:");
//    Serial.print(deltaTheta);

    //閾値内にあるときは真っ直ぐ
    if ((0 <= deltaTheta && deltaTheta <= this->threshold / 2) || (360 - this->threshold / 2 <= deltaTheta && deltaTheta <= 360)) {
      motor.stop();
    }
    //閾値よりプラスで大きい時は反時計回りに回るようにする（右が速くなるようにする）
    else if (this->threshold / 2 < deltaTheta && deltaTheta <= 180) {
      motor.spinLeft(deltaTheta * 120 / 180);//"spin left"
    }

    //閾値よりマイナスで大きい時は時計回りに回るようにする（左が速くなるようにする）
    else {
      motor.spinRight(deltaTheta * 120 / 180);//"spin right"

    }
  }
  return;
}
