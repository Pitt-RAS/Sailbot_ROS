#include "config.h"
#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface():
    r9(TX_SERIALPORT), watchdog(TX_TIMEOUT), sailAngle(0), rudderAngle(0), enabled(false), autonomous(false) {
    r9.begin();
}

void TransmitterInterface::update() {
    if ( r9.read(&channels[0], &failSafe, &lostFrames) ) {
        enabled = channels[4] > 1500;
        autonomous = channels[5] > 1500;

        watchdog.feed();

        rudderAngle = map(channels[1],172,1808,0,180);
        rudderAngle = abs(rudderAngle - 180); //flips
        sailAngle = map(channels[0],172,1811,0,70);

        sailAngle = channels[0]-172;
        sailAngle /= 1640;
        sailAngle *= 8000;

        rudderAngle = channels[1]-1000;
        rudderAngle /= 828;
        rudderAngle *= -90;
    }
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

