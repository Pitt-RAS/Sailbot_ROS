#ifndef TRANSMITTER_INTERFACE_H
#define TRANSMITTER_INTERFACE_H

class TransmitterInterface {
public:
    TransmitterInterface();

    double getSailAngle();
    double getRudderAngle();

    bool wantsEnable();
    bool wantsAutonomous();
    bool wantsTeleop();
};

#endif
