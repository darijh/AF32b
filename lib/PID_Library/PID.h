
#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(double kp, double ki, double kd, bool reverse = false,
        double outMin = 0, double outMax = 255, unsigned long sampleTimeMs = 10);

    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double min, double max);
    void setSampleTime(unsigned long newSampleTimeMs);
    void setMode(bool enable);
    void setDirection(bool reverse);
    void reset();

    double compute(double input, double setpoint);
    bool isSaturated() const;

  private:
    double kp, ki, kd;
    double outMin, outMax;
    double integral, prevError;
    bool enabled;
    bool reverseDirection;
    bool saturated;
    unsigned long lastTime;
    unsigned long sampleTime;
};

#endif
