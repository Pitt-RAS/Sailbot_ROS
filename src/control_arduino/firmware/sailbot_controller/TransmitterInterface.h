#ifndef TRANSMITTER_INTERFACE_H
#define TRANSMITTER_INTERFACE_H

class TransmitterInterface {
public:
    TransmitterInterface();

    double getSailAngle();
    double getRudderAngle();

    bool wantsEnable();
    bool wantsAutonomous();
    
    void update();
    
private:

    double sailAngle;
    double rudderAngle;
    bool enabled;
    bool autonomous;
};

#endif
