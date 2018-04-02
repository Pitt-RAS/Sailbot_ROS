#include "Rate.h"
#include <Arduino.h>

Rate::Rate(int hz) {
    setRate(hz);
    lastUs = 0;
}

void Rate::setRate(int hz) {
    periodUs = (1.0/(double)hz) * 1000000;
}

bool Rate::needsRun() {
    unsigned long dt = micros() - lastUs;
    if ( dt >= periodUs ) {
        lastUs = micros();
        return true;
    }
    return false;
}

void Rate::sleep() {
    unsigned long dt = micros() - lastUs;

    if ( dt < periodUs )
        delayMicroseconds(periodUs-dt);

    lastUs = micros();
}
