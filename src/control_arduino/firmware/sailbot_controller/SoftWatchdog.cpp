#include "SoftWatchdog.h"
#include <Arduino.h>

SoftWatchdog::SoftWatchdog(unsigned int maxAge) {
    lastFed = 0;
    this->maxAge = maxAge;
}

void SoftWatchdog::feed() {
    lastFed = millis();
}

bool SoftWatchdog::hungry() {
    return millis() - lastFed >= maxAge;
}

