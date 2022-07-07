#ifndef _MOTOR_H
#define _MOTOR_H

class Motor{
private:
    const uint8_t ENABLE = 8;
    const uint8_t CH1 = 9;
    const uint8_t CH2 = 11;
    const uint8_t CH3 = 10;
    const uint8_t CH4 = 12;
public:
    int controlStatus;
    int speedL;
    int speedR;
    int threshold = 10; //角度の差分の閾値
    int spinThreshold = 10; //角度の差分の閾値
    float deltaTheta;//目的方向と姿勢の相対角度差

    Motor(uint8_t enable,uint8_t ch1,uint8_t ch2,uint8_t ch3,uint8_t ch4);
    void init();
    void setThreshold(int threshold,int spinthreshold);
    void stop();
    void goStraight(int speed);
    void turn(int speedl,int speedr);
    void spinLeft(int speed);
    void spinRight(int speed);
    void angleGo(float bodyDeg,float goalDeg,int speed);
};


#endif
