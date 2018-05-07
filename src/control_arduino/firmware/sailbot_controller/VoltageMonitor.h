#ifndef VOLTAGE_MONITOR_H
#define VOLTAGE_MONITOR_H

#include <ros.h>
#include <std_msgs/Float32.h>

class VoltageMonitor {
public:
    VoltageMonitor(ros::NodeHandle* _nh);

    double getVoltage();
    void update();
private:
    ros::NodeHandle* nh;
    std_msgs::Float32 volt_msg;
    ros::Publisher* batVoltPub;
};

#endif
