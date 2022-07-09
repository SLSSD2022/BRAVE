#ifndef _GPS_H_
#define _GPS_H_
#include <HardwareSerial.h>
#include <TinyGPS++.h>

class GPS : public TinyGPSPlus{
private:
    HardwareSerial* HWSerial;
    //静的メンバ変数
    //a, F: 世界測地系-測地基準系1980（GRS80）楕円体
    static float m0;
    static long a;
    static float F;
    //n, A_i, alpha_iの計算
    static float n;
    static float A_0;
    static float A_1;
    static float A_2;
    static float A_3;
    static float A_4;
    static float A_5;
    static float a1;
    static float a2;
    static float a3;
    static float a4;
    static float a5;
    static float A_;
    
public:
    //基準点に依存する定数
    float phi0_deg;
    float lambda0_deg;
    float phi0_rad;
    float lambda0_rad;
    float mid_S_;
    float S_;

    
    GPS();
    GPS(HardwareSerial *serialport);
    void init();
    void updateGPSlocation(float* lat,float* lng);

    float deg2rad(float deg);
    float atanh(float x);
    int calc_const(float phi_deg, float lambda_deg);
    void calc_xy(float phi_deg, float lambda_deg ,float* x,float* y);
};


#endif 
