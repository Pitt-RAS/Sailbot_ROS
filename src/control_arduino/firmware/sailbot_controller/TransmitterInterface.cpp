#include "config.h"
#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface():
    sailAngle(0), rudderAngle(0), enabled(false), autonomous(false), r9(TX_SERIALPORT), watchdog(TX_TIMEOUT) {
    r9.begin();
}

void TransmitterInterface::update() {
    if ( r9.read(&channels[0], &failSafe, &lostFrames) ) {
        enabled = channels[4] > 1500;
        autonomous = false;
        watchdog.feed();
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
    return enabled && isConnected();
}

bool TransmitterInterface::wantsAutonomous() {
    return autonomous && isConnected();
}

bool TransmitterInterface::isConnected() {
    return !watchdog.hungry() && !failSafe;
}
