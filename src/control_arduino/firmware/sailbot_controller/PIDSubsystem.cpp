#include "Arduino.h"
#include "PIDSubsystem.h"

PIDSubsystem::PIDSubsystem(int analogSensorPin, int pwmPin)
    : pwm(pwmPin) {
    this->analogSensorPin = analogSensorPin;
}

PIDSubsystem::PIDSubsystem(int analogSensorPin, int pwmPin, double adcOffset, double adcConversion)
    : PIDSubsystem(analogSensorPin, pwmPin)
{
    this->adcOffset = adcOffset;
    this->adcConversion = adcConversion;
}

void PIDSubsystem::configSetpointUnits(double adcOffset, double adcConversion) {
    this->adcOffset = adcOffset;
    this->adcConversion = adcConversion;
}

void PIDSubsystem::setSetpoint(double setpoint) {
    setpoint *= adcConversion;
    setpoint += adcOffset;

    setRawSetpoint(setpoint);
}

void PIDSubsystem::setRawSetpoint(int setpoint) {
    controlActive = true;

}

void PIDSubsystem::setOpenLoop(double speed) {
    controlActive = false;
    pwm.set(speed);
}

void PIDSubsystem::update() {
    if ( !controlActive )
        return;
    int actual = analogRead(analogSensorPin);
    // TODO: Implement PID controller
}

