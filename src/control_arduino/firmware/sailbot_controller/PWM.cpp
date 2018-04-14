#include "PWM.h"
#include <math.h>

PWM::PWM(int pin) {
    pwm.attach(pin);
    set(0);
}

void PWM::set(double speed) {
    if ( fabs(speed) > 1 )
        speed = copysign(1, speed);
    speed *= 500;
    speed += 1500;

    pwm.writeMicroseconds((int)speed);
}

