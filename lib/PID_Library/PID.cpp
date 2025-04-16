
#include "PID.h"
#include <Arduino.h>

PID::PID(double kp_, double ki_, double kd_, bool reverse, double outMin_, double outMax_, unsigned long sampleTimeMs) {
  setTunings(kp_, ki_, kd_);
  setOutputLimits(outMin_, outMax_);
  setSampleTime(sampleTimeMs);
  setDirection(reverse);
  setMode(true);
  reset();
}

void PID::setTunings(double kp_, double ki_, double kd_) {
  kp = kp_;
  ki = ki_;
  kd = kd_;
}

void PID::setOutputLimits(double min, double max) {
  outMin = min;
  outMax = max;
}

void PID::setSampleTime(unsigned long newSampleTimeMs) {
  sampleTime = newSampleTimeMs;
}

void PID::setMode(bool enable) {
  enabled = enable;
}

void PID::setDirection(bool reverse) {
  reverseDirection = reverse;
}

void PID::reset() {
  integral = 0;
  prevError = 0;
  lastTime = millis();
  saturated = false;
}

double PID::compute(double input, double setpoint) {
  if (!enabled) return -1;

  unsigned long now = millis();
  unsigned long deltaTime = now - lastTime;
  if (deltaTime < sampleTime) return -1;

  lastTime = now;

  double error = setpoint - input;
  if (reverseDirection) error = -error;

  integral += error * (deltaTime / 1000.0);
  double derivative = (error - prevError) / (deltaTime / 1000.0);
  prevError = error;

  double output = kp * error + ki * integral + kd * derivative;

  saturated = false;
  if (output > outMax) {
    output = outMax;
    saturated = true;
    integral -= error * (deltaTime / 1000.0);
  } else if (output < outMin) {
    output = outMin;
    saturated = true;
    integral -= error * (deltaTime / 1000.0);
  }

  return output;
}

bool PID::isSaturated() const {
  return saturated;
}
