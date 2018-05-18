#include <iostream>
#include <string>
#include <unistd.h>
#include "XbeeReceiver.h"
#include "XbeeSender.h"
#include "NonblockingRate.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/ioctl.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "xbee_receiver");
    ros::NodeHandle nh;

    std::string portName;
    nh.param<std::string>("port", portName, "/dev/ttyACM0");
    int sock = open(portName.c_str(), O_RDWR);

    if ( sock == -1 ) {
        perror("open()");
        return 1;
    }

    NonblockingRate sendRate(5);

    XbeeReceiver xbeeReceiver(nh, sock);
    XbeeSender xbeeSender(nh, sock);

    while ( nh.ok() ) {
        xbeeReceiver.update();
        if ( sendRate.needsRun() ) {
            xbeeSender.update();
        }
        ros::spinOnce();
    }
    return 0;
}

