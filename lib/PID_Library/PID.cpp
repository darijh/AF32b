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
  _integral += error;

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

  double output = _kp * error + _ki * _integral + _kd * derivative;

  if (output > _outputMax)
    output = _outputMax;
  else if (output < _outputMin)
    output = _outputMin;

  return output;
}