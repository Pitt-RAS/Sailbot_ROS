#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface(): 
    sailAngle(0), rudderAngle(0), enabled(false), autonomous(false), r9(Serial1) {
    
}

void TransmitterInterface::update() {
    if ( r9.read(&channels[0], &failSafe, &lostFrames) ) {
        enabled = channels[4] > 1500;
        autonomous = false;
    }

//    if (sa >= 0 && sa <= 180)
//        sailAngle = sa;
//    if (ra >= 0 && ra <= 180)
//        rudderAngle = ra;
//    enabled = e;
//    autonomous = a;
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

