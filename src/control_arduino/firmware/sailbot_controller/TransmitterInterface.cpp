#include <Arduino.h>
#include "config.h"
#include "TransmitterInterface.h"

TransmitterInterface::TransmitterInterface():
    watchdog(TX_TIMEOUT), startPktBuffer(0), gotStart(false), bufPos(0), sailAngle(0), rudderAngle(0), enabled(false), autonomous(false) {
}

void TransmitterInterface::update() {
    if ( XBEE_SERIALPORT.available() > 0 ) {
        uint8_t buf = XBEE_SERIALPORT.read();

        startPktBuffer = (uint32_t)startPktBuffer >> 8;
        uint32_t temp = (uint32_t)buf << 24;
        startPktBuffer = startPktBuffer | temp;
        if ( startPktBuffer == XBEE_START_VAL ) {
            bufPos = 0;
            gotStart = true;
        }
        else if ( bufPos == sizeof(TransmitterInterfacePacket) ) {
            if ( gotStart ) {
                enabled = packet.enabled;
                autonomous = packet.autonomous;
                sailAngle = packet.sailCommand;
                rudderAngle = packet.rudderCommand;

                watchdog.feed();
            }
            bufPos = 0;
            gotStart = false;
        }
        else {
            ((char*)&packet)[bufPos++] = buf;
        }
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
    return !watchdog.hungry();
}

