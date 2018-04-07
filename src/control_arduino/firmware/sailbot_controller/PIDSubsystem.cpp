#include "config.h"
#include <Arduino.h>
#include "PIDSubsystem.h"

PIDSubsystem::PIDSubsystem(const char* name, int _analogSensorPin, int pwmPin, double _adcOffset, double _adcConversion, double kP, double kI, double kD, ros::NodeHandle* _nh)
    : nh(_nh), analogSensorPin(_analogSensorPin), pwm(pwmPin), pid(kP, kI, kD), adcOffset(_adcOffset), adcConversion(_adcConversion) {

    if ( shouldUseROS ) {
        char topic[64];
        sprintf(topic, "/pidsubsystem/%s/p", name);
        pConfigSub = new ros::Subscriber<std_msgs::Float64, PIDSubsystem>(strdup(topic), &PIDSubsystem::updatePTerm, this);
        sprintf(topic, "/pidsubsystem/%s/i", name);
        iConfigSub = new ros::Subscriber<std_msgs::Float64, PIDSubsystem>(strdup(topic), &PIDSubsystem::updateITerm, this);
        sprintf(topic, "/pidsubsystem/%s/d", name);
        dConfigSub = new ros::Subscriber<std_msgs::Float64, PIDSubsystem>(strdup(topic), &PIDSubsystem::updateDTerm, this);

        nh->subscribe(*pConfigSub);
        nh->subscribe(*iConfigSub);
        nh->subscribe(*dConfigSub);
    }
}

PIDSubsystem::PIDSubsystem(const char* name, int analogSensorPin, int pwmPin, double kP, double kI, double kD, ros::NodeHandle* _nh)
    : PIDSubsystem(name, analogSensorPin, pwmPin, 0, 1, kP, kI, kD, _nh) {}

void PIDSubsystem::configGains(double kP, double kI, double kD) {
    pid.configGains(kP, kI, kD);
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
    pwm.set(0);
}

void PIDSubsystem::setOpenLoop(double speed) {
    controlActive = false;
    pwm.set(speed);
}

void PIDSubsystem::updatePTerm(const std_msgs::Float64& pTerm) {
    pid.configP(pTerm.data);
}

void PIDSubsystem::updateITerm(const std_msgs::Float64& iTerm) {
    pid.configI(iTerm.data);
}

void PIDSubsystem::updateDTerm(const std_msgs::Float64& dTerm) {
    pid.configD(dTerm.data);
}

double PIDSubsystem::getActual() {
    return actualPosition;
}

void PIDSubsystem::update() {
    if ( !controlActive )
        return;
    int actual = analogRead(analogSensorPin);
    actualPosition = (actual-adcOffset) / adcConversion;
    double output = pid.calculate(actual);
    pwm.set(output);
}

