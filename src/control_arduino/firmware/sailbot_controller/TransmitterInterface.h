#ifndef TRANSMITTER_INTERFACE_H
#define TRANSMITTER_INTERFACE_H

#include <ros.h>
#include <SBUS.h>
#include "SoftWatchdog.h"
#include <visualization/BoatState.h>

class TransmitterInterface {
public:
    TransmitterInterface(ros::NodeHandle* _nh);

    double getSailAngle();
    double getRudderAngle();

    bool wantsEnable();
    bool wantsAutonomous();
    bool isConnected();
    bool wantsFirstPreset();

    visualization::BoatState boatStateMsg;
    ros::Publisher* boatStatePub;

    void update();
private:
    ros::NodeHandle *nh;
    SBUS r9;
    SoftWatchdog watchdog;
    uint16_t channels[16];
    uint8_t failSafe;
    uint16_t lostFrames = 0;

    double sailAngle;
    double rudderAngle;
    bool enabled;
    bool autonomous;
    bool presetOne;
};

#endif
