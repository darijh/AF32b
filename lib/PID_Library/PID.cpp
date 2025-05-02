#include "PID.h"

PID::PID(double kp, double ki, double kd)
    : _kp(kp), _ki(ki), _kd(kd), _integral(0), _prevError(0), _outputMin(0),
      _outputMax(10000), _firstCompute(true) {}

void PID::setOutputLimits(double min, double max)
{
  _outputMin = min;
  _outputMax = max;
}

void PID::reset()
{
  _integral = 0;
  _prevError = 0;
  _firstCompute = true;
}

double PID::compute(double input, double setpoint)
{
  double error = setpoint - input;
  _integral = constrain(_integral + error * _ki * .002, _outputMin, _outputMax);

  double derivative = 0;
  if (!_firstCompute)
  {
    derivative = error - _prevError;
  }
  else
  {
    _firstCompute = false;
  }

  _prevError = error;
  // Calculate the output using PID formula
  double output = _kp * error + _integral + (_kd / .002) * derivative;
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" Integral: ");
  Serial.print(_integral);
  Serial.print(" Derivative: ");
  Serial.print(derivative);
  Serial.print(" Output: ");
  Serial.println(output);

  if (output > _outputMax)
    output = _outputMax;
  else if (output < _outputMin)
    output = _outputMin;

  return output;
}