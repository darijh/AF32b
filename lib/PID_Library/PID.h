#ifndef PID_H
#define PID_H
#include <Arduino.h>
class PID
{
public:
  PID(double kp, double ki, double kd, double time_step);

  void setOutputLimits(double min, double max);
  void setEnable(bool enable);
  void AlarmConfig(unsigned long timeout, double threshold);
  double compute(double input, double setpoint);
  bool getAlarm();
  void reset();

private:
  double _kp, _ki, _kd, _time_step;
  double _integral;
  double _prevError;
  double _outputMin, _outputMax;
  bool _firstCompute;
  bool _enable;
  bool _alarm;
  unsigned long _alarm_timeout;
  double _alarm_threshold;
  unsigned long _ml_alarm;
};

#endif