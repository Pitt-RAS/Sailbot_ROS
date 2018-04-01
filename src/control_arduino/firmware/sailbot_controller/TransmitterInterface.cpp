#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface(): 
    sailAngle(0), rudderAngle(0), enabled(false), autonomous(false) {
    
}

void TransmitterInterface::update(double sa, double ra, bool e, bool a) {
    if (sa >= 0 && sa <= 180)
        sailAngle = sa;
    if (ra >= 0 && ra <= 180)
        rudderAngle = ra;
    enabled = e;
    autonomous = a;
}

double TransmitterInterface::getSailAngle() {
    return sailAngle;
}

double TransmitterInterface::getRudderAngle() {
    return rudderAngle;
}

bool TransmitterInterface::wantsEnable() {
    return enabled;
}

bool TransmitterInterface::wantsAutonomous() {
    return autonomous;
}

