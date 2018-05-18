#ifndef NONBLOCKING_RATE_H
#define NONBLOCKING_RATE_H

#include <ros/ros.h>

class NonblockingRate {
public:
    NonblockingRate(int hz);
    bool needsRun();
private:
    ros::Duration period;
    ros::Time nextRun;
};

#endif

