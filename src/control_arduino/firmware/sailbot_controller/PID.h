#ifndef PID_H
#define PID_H

class PID {
    double kP;
    double kI;
    double kD;
    double setpoint;
    double integral;
    double lastTime;
    double lastError;
    bool firstIter;

public:

    PID(double kP, double kI, double kD);
    void configGains(double kP, double kI, double kD);
    void configP(double kP);
    void configI(double kI);
    void configD(double kD);
    void setSetpoint(double setpoint);
    double calculate(double actual);
    double now();
};

#endif

