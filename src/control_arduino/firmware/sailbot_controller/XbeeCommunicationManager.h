#ifndef XBEE_COMM_MGR_H
#define XBEE_COMM_MGR_H

#include <ros/ros.h>

class XbeeCommunicationManager {
public:
    XbeeCommunicationManager(ros::NodeHandle* nh);
    void update();
};

#endif

