#include "PID.h"

PID::PID(double kp, double ki, double kd, double time_step)
    : _kp(kp), _ki(ki), _kd(kd), _time_step(time_step), _integral(0), _prevError(0), _outputMin(0),
      _outputMax(10000), _firstCompute(true), _enable(0), _alarm(0), _alarm_timeout(20000), _alarm_threshold(10), _ml_alarm(0) {}

void PID::setOutputLimits(double min, double max)
{
  if (max > min)
  {
    _outputMin = min;
    _outputMax = max;
  }
}

void PID::setEnable(bool enable)
{
  _enable = enable;
  if (!enable)
  {
    reset();
  }
}

void PID::AlarmConfig(unsigned long timeout, double threshold)
{
  _alarm_timeout = timeout;
  _alarm_threshold = threshold;
}

double PID::compute(double input, double setpoint)
{
  if (!_enable)
  {
    return 0;
  }

  double error = setpoint - input;
  double relative_error = 0;
  if (setpoint)
  {
    double relative_error = abs(error) * 100 / setpoint;
  }

  if (relative_error < _alarm_threshold)
  {
    _ml_alarm = millis();
    _alarm = false;
  }
  else if ((millis() - _ml_alarm) > _alarm_timeout)
  {
    _alarm = true;
  }

  _integral = constrain(_integral + (error * _ki * _time_step), _outputMin, _outputMax);

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

  double output = _kp * error + _integral + (_kd / _time_step) * derivative;

  if (output > _outputMax)
    output = _outputMax;
  else if (output < _outputMin)
    output = _outputMin;

  return output;
}

bool PID::getAlarm()
{
  return _alarm;
}

void PID::reset()
{
  _integral = 0;
  _prevError = 0;
  _firstCompute = true;
}
