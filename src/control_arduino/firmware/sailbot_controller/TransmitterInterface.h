#ifndef TRANSMITTER_INTERFACE_H
#define TRANSMITTER_INTERFACE_H

#include "SoftWatchdog.h"
#include <stdint.h>

struct TransmitterInterfacePacket {
    bool enabled;
    bool autonomous;
    int16_t sailCommand;
    int16_t rudderCommand;
} __attribute__((packed));

class TransmitterInterface {
public:
    TransmitterInterface();

    double getSailAngle();
    double getRudderAngle();

    bool wantsEnable();
    bool wantsAutonomous();
    bool isConnected();

    void update();

private:
    SoftWatchdog watchdog;

    TransmitterInterfacePacket packet;
    int32_t startPktBuffer;
    bool gotStart;
    int bufPos;

    double sailAngle;
    double rudderAngle;
    bool enabled;
    bool autonomous;
};

#endif
