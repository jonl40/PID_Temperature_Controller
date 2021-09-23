#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID
{
  public:
    // PID constructor init values
    PID(float KpPID, float kiPID, float KdPID,
        float TauPID,
        float integratorMaxPID, float integratorMinPID,
        float outMaxPID, float outMinPID,
        float SampleTime,
        float spPID);

    // compute control signal given current measurement 
    float control(float measure);

  private:
    // pid coefficients 
    float kp;
    float ki;
    float kd;

    // low pass filter constant 
    float tau; 

    // integratal limits to negate integrator wind up 
    float integralMax;
    float integralMin;

    // output limits 
    float outputMax;
    float outputMin; 

    // sampling time (milliseconds) 
    float T;

    // controller variables
    float setpoint;
    float integral;
    float derivative;
    float prevError; 
    float prevMeasure;
    /* PID output */
    float controlSignal; 
};


class Heater
{
  public: 
    Heater(int pinNum);
    void pwm(int value);

  private: 
    int pin; 
};

#endif 
