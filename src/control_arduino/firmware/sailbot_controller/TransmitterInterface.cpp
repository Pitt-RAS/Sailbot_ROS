#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface() {
}

double TransmitterInterface::getSailAngle() {
    return 0;
}

double TransmitterInterface::getRudderAngle() {
    return 0;
}

bool TransmitterInterface::wantsEnable() {
    return false;
}

bool TransmitterInterface::wantsAutonomous() {
    return false;
}


bool TransmitterInterface::wantsTeleop() {
    return false;
}

