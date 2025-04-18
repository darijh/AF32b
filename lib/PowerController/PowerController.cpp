#include "PowerController.h"
#include <Arduino.h>

PowerController::PowerController(
    double voltageKp, double voltageKi, double voltageKd,
    double currentKp, double currentKi, double currentKd)
    : _pidVoltage(voltageKp, voltageKi, voltageKd),
      _pidCurrent(currentKp, currentKi, currentKd),
      _voltageSetpoint(0),
      _currentSetpoint(0),
      _restCurrent(0),
      _restMode(false),
      _currentMode(false),
      _voltageDuty(0),
      _currentDuty(0)
{
    _pidVoltage.setOutputLimits(0, 10000);
    _pidCurrent.setOutputLimits(0, 10000);
    _lastStableMillis = millis();
}

void PowerController::setSetpoints(double voltageSetpoint_mV, double currentSetpoint_mA, double restCurrent_mA)
{
    _voltageSetpoint = voltageSetpoint_mV;
    _currentSetpoint = currentSetpoint_mA;
    _restCurrent = restCurrent_mA;
}

void PowerController::update(double voltageReading, double currentReading, bool restMode)
{
    _restMode = restMode;
    _voltageReading = voltageReading;
    _currentReading = currentReading;

    double activeCurrentSetpoint = _restMode ? _restCurrent : _currentSetpoint;

    double errorV = relativeError(voltageReading, _voltageSetpoint);
    double errorI = relativeError(currentReading, activeCurrentSetpoint);

    // Lógica de modo
    if (_currentMode)
    {
        if (errorI > 0.08 && errorV < 0.02)
        {
            _currentMode = false;
            _pidCurrent.reset();
        }
    }
    else
    {
        if (errorI < _hysteresis)
        {
            _currentMode = true;
            _pidVoltage.reset();
        }
    }

    // PID
    if (_currentMode)
    {
        _currentDuty = static_cast<int>(_pidCurrent.compute(currentReading, activeCurrentSetpoint));
        _voltageDuty = 10000;
    }
    else
    {
        _voltageDuty = static_cast<int>(_pidVoltage.compute(voltageReading, _voltageSetpoint));
        _currentDuty = 10000;
    }

    // Diagnóstico
    double currentError = _currentMode ? errorI : errorV;
    if (currentError > 0.2)
    {
        if ((millis() - _lastStableMillis) > _unstableTimeout)
        {
            _unstableFlag = true;
        }
    }
    else
    {
        _lastStableMillis = millis();
        _unstableFlag = false;
    }
}

bool PowerController::isCurrentMode() const
{
    return _currentMode;
}

bool PowerController::isRestMode() const
{
    return _restMode;
}

int PowerController::getVoltageDuty() const
{
    return _voltageDuty;
}

int PowerController::getCurrentDuty() const
{
    return _currentDuty;
}

bool PowerController::isNotReachingSetpoint()
{
    return _unstableFlag;
}

void PowerController::setUnstableTimeout(unsigned long ms)
{
    _unstableTimeout = ms;
}

double PowerController::relativeError(double measured, double setpoint)
{
    if (setpoint == 0)
        return 0;
    return std::abs(setpoint - measured) / setpoint;
}
