#ifndef PID_H
#define PID_H

#include <Arduino.h>

// heater pin
#define PWM_PIN 28


// pid constants 
#define KP_PID              0.1f
#define kI_PID              0.1f
#define KD_PID              0.1f
#define TAU_PID             0.01f
#define INTEGRATE_MAX_PID   128.0f
#define INTEGRATE_MIN_PID   0.0f
#define OUT_MAX_PID         255.0f
#define OUT_MIN_PID         0.0f  
#define SAMPLE_TIME         1000.0f
#define SP_PID              30.0f


class PID
{
  public:
    // PID constructor init values
    PID(float KpPID, float kiPID, float KdPID,
        float TauPID,
        float integrateMaxPID, float integrateMinPID,
        float outMaxPID, float outMinPID,
        float SampleTime,
        float spPID);

    // compute control signal given current measurement 
    void control(float measure);

    // return integer controlSignal (pwm takes integer values)
    int out();

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
