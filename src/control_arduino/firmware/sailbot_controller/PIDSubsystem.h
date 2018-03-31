#ifndef PIDSUBSYSTEM_H
#define PIDSUBSYSTEM_H

#include "PWM.h"

class PIDSubsystem {
public:
    PIDSubsystem(int analogSensorPin, int pwmPin);
    PIDSubsystem(int analogSensorPin, int pwmPin, double adcOffset, double adcConversion);

    void configSetpointUnits(double adcOffset, double adcConversion);

    void setRawSetpoint(int setpoint);
    void setSetpoint(double setpoint);
    void setOpenLoop(double speed);

    void update();
private:
    int analogSensorPin;
    PWM pwm;
    bool controlActive;
    double adcOffset;
    double adcConversion;
};

#endif
