#include "PWM.h"
#include <math.h>

PWM::PWM(int pin) {
    pwm.attach(pin);
    set(0);
    configLimit(1);
}

void PWM::configLimit(double limit) {
    this->limit = limit;
}

void PWM::set(double speed) {
    if ( fabs(speed) > 1 )
        speed = copysign(1, speed);
    speed *= limit;
    speed *= 500;
    speed += 1500;

    pwm.writeMicroseconds((int)speed);
}

