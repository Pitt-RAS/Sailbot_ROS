#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface(): 
    sailAngle(0), rudderAngle(0), wantsEnable(false), wantsAutonomous(false) {
    
}

void update(double sa, double ra, bool e, bool a) {
    if (0 <= sa <= 180)
        sailAngle = sa;
    if (0 <= ra <= 180)
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
