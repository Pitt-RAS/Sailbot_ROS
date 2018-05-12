#include <iostream>
#include <unistd.h>
#include "XbeeReceiver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "xbee_receiver");
    ros::NodeHandle nh;

    XbeeReceiver xbee(nh);
    while ( nh.ok() ) {
        xbee.update();
    }
    return 0;
}

