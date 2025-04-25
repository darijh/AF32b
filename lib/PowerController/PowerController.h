#ifndef POWER_CONTROLLER_H
#define POWER_CONTROLLER_H

#include "PID.h"

class PowerController {
public:
  PowerController(double voltageKp, double voltageKi, double voltageKd,
                  double currentKp, double currentKi, double currentKd);

  void setSetpoints(double voltageSetpoint_dV, double currentSetpoint_mA,
                    double restCurrent_mA);
  void update(double voltageReading, double currentReading, bool restMode);
  void setEnable(bool enable);

  int getVoltageDuty() const;
  int getCurrentDuty() const;

  bool isCurrentMode() const;
  bool isRestMode() const;
  bool isNotReachingSetpoint();
  void setUnstableTimeout(unsigned long ms); // Ajuste del tiempo de diagnóstico

private:
  PID _pidVoltage;
  PID _pidCurrent;

  double _voltageSetpoint;
  double _currentSetpoint;
  double _restCurrent;

  bool _restMode;
  bool _currentMode;
  bool _enable = false;

  double _voltageReading;
  double _currentReading;

  int _voltageDuty;
  int _currentDuty;

  const double _hysteresis = 0.03;

  // Diagnóstico
  unsigned long _lastStableMillis = 0;
  unsigned long _unstableTimeout = 60000; // por defecto 1 minuto
  bool _unstableFlag = false;

  double relativeError(double measured, double setpoint);
};

#endif