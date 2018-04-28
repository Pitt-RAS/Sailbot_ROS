#ifndef PWM_H
#define PWM_H

#include <Servo.h>

class PWM {
public:
    PWM(int pin);
    void set(double speed);
    void configLimit(double limit);
private:
    Servo pwm;
    double limit;
};

#endif
