#ifndef _MOTOR_H
#define _MOTOR_H

class Motor{
private:
    const uint8_t ENABLE = 3;
    const uint8_t CH1 = 4;
    const uint8_t CH2 = 5;
    const uint8_t CH3 = 6;
    const uint8_t CH4 = 7;
public:
    int controlStatus;
    int powerL;
    int powerR;
    int threshold = 10; //角度の差分の閾値
    int spinThreshold = 6; //角度の差分の閾値
    float deltaTheta;//目的方向と姿勢の相対角度差
    float deltaLR;//左右差の補正項

    Motor(uint8_t enable,uint8_t ch1,uint8_t ch2,uint8_t ch3,uint8_t ch4);
    void init();
    void setThreshold(int threshold,int spinthreshold);
    void stop();
    void right(int power);
    void left(int power);
    void goStraight(int power);
    void turn(int powerL,int powerR);
    void spinLeft(int power);
    void spinRight(int power);
    void angleGo(float bodyDeg,float goalDeg,int power);  
    void angleSpin(float bodyDeg,float goalDeg);
};


#endif
