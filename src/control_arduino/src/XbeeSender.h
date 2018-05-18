#ifndef XBEE_SENDER_H
#define XBEE_SENDER_H

#include <ros/ros.h>
#include "common.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sensor_msgs/Joy.h>


struct TransmitterInterfacePacket {
    int32_t start;
    bool enabled;
    bool autonomous;
    int16_t sailCommand;
    int16_t rudderCommand;
} __attribute__((packed));

class XbeeSender {
    public:
        XbeeSender(ros::NodeHandle& _nh, int fd);
        void update();

    private:
        TransmitterInterfacePacket packet;

        ros::NodeHandle& nh;
        int file;
        ros::Subscriber joySub;

        void updateJoystick(const sensor_msgs::Joy::ConstPtr& js);
};

#endif
