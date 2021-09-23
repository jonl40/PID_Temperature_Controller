#include "PID.h"

PID::PID(float KpPID, float kiPID, float KdPID,
         float TauPID,
         float integratorMaxPID, float integratorMinPID,
         float outMaxPID, float outMinPID,
         float SampleTime,
         float spPID)
{
  // pid coefficients
  kp = KpPID;
  ki = kiPID;
  kd = KdPID;

  // low pass filter constant
  tau = TauPID;

  // integral limits to negate integrator wind up 
  integralMax = integratorMaxPID; 
  integralMin = integratorMinPID;

  // output limits 
  outputMax = outMaxPID;
  outputMin = outMinPID;

  // sampling time (milliseconds) 
  T = SampleTime;

  // desired system output 
  setpoint = spPID;

  // initial controller values 
  integral = 0.0f;
  derivative = 0.0f; 
  prevError = 0.0f;
  prevMeasure = 0.0f;
}


float PID::control(float measure)
{
  float error = setpoint - measure;

  float proportional = kp * error; 
  integral = ((ki * T) / 2.0f) * (error - prevError) + integral; 
  derivative = ((-2.0f * kd)/(2.0f * tau + T)) * (measure - prevMeasure) + ((2.0f * tau - T)/(2.0f * tau + T)) * derivative;

  if (integral > integralMax)
  {
    integral = integralMax;
  }
  else if (integral < integralMin)
  {
    integral = integralMin;
  }

  controlSignal = proportional + integral + derivative;

  if (controlSignal > outputMax)
  {
    controlSignal = outputMax;
  }
  else if (controlSignal < outputMin) 
  {
    controlSignal = outputMin;
  }
}


Heater::Heater(int pinNum)
{
  pin = pinNum;
}


void Heater::pwm(int value)
{
  analogWrite(pin, value);
}
