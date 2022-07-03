#ifndef MOTOR_H
#define MOTOR_H

class Motor{
private:
    const uint8_t ENABLE = 8;
    const uint8_t CH1 = 9;
    const uint8_t CH2 = 11;
    const uint8_t CH3 = 10;
    const uint8_t CH4 = 12;
public:
    int threshold = 10; //角度の差分の閾値
    float deltaTheta;//目的方向と姿勢の相対角度差
    int spinThreshold = 12; //純粋なスピン制御を行う角度を行う閾値(スピンで機軸変更する時のみ)
    int nominalSpeed = 250;
    int slowSpeed = 200;
    int verySlowSpeed = 150;
}


#endif