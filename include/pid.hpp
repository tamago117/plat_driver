#pragma once

#include <Arduino.h>

class PID
{
public:
    PID(float Kp_, float Ki_, float Kd_);
	double pid_controll(double goal, double presentValue);
    void resetI();

private:
    float Kp;
    float Ki;
    float Kd;
    uint32_t preTime;
    double preP = 0;
    double I = 0;
  
};

PID::PID(float Kp_, float Ki_, float Kd_){
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    preTime = micros();
}

double PID::pid_controll(double goal, double presentValue)
{
	uint32_t dt = micros() - preTime;
	preTime = micros();

	double P = goal - presentValue;
	I += P * dt;
	double D = (P - preP) / dt;

    preP = P;
  
	return _Kp*P + _Ki*I + _Kd*D ;
}

void PID::resetI()
{
    I = 0;
}
