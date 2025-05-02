#ifndef PID_H
#define PID_H
#include <Arduino.h>
class PID
{
public:
  PID(double kp, double ki, double kd);

  void setOutputLimits(double min, double max);
  void reset();
  double compute(double input, double setpoint);

private:
  double _kp, _ki, _kd;
  double _integral;
  double _prevError;
  double _outputMin, _outputMax;
  bool _firstCompute;
};

#endif