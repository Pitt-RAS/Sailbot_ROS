#include "PID.h"
#include <Arduino.h>

PID::PID(double kP, double kI, double kD) {
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;
    this->setpoint = 0;
    this->integral = 0;
    this->firstIter = true;
    this->lastTime = now();
}

void PID::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

double PID::now() {
    double time = millis()/1000.0;
    return time;
}

double PID::calculate(double actual) {
    double dt = now() - lastTime;
    lastTime = now();

    double error = setpoint - actual;

    integral += error * dt;

    double dEdT = 0;
    if(firstIter){
        firstIter = false;
    } else {
        dEdT = (error - lastError) / dt;
    }
    
    lastError = error;

    return error*kP + integral*kI + dEdT*kD;

}

