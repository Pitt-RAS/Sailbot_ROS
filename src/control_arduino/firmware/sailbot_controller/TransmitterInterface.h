#ifndef TRANSMITTER_INTERFACE_H
#define TRANSMITTER_INTERFACE_H

#include <SBUS.h>

class TransmitterInterface {
public:
    TransmitterInterface();

    double getSailAngle();
    double getRudderAngle();

    bool wantsEnable();
    bool wantsAutonomous();
    
    void update();
    
private:
    SBUS r9;
    uint16_t channels[16];
    uint8_t failSafe;
    uint16_t lostFrames = 0;

    double sailAngle;
    double rudderAngle;
    bool enabled;
    bool autonomous;
};

#endif
