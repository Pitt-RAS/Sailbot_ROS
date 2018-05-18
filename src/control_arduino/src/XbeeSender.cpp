#include "XbeeSender.h"

XbeeSender::XbeeSender(ros::NodeHandle& _nh, int fd) :
    nh(_nh),
    file(fd),
    joySub(nh.subscribe<sensor_msgs::Joy>("/joy", 10, &XbeeSender::updateJoystick, this)) {

        memset(&packet, 0, sizeof(TransmitterInterfacePacket));
        packet.start = XBEE_STARTBIT;
}

void XbeeSender::updateJoystick(const sensor_msgs::Joy::ConstPtr& js) {
    // TODO: JS conversion
}

void XbeeSender::update() {
    write(file, &packet, sizeof(TransmitterInterfacePacket));
}

