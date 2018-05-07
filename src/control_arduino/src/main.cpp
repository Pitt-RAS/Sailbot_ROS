#include <iostream>
#include <unistd.h>
#include "XbeeReceiver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odom");
    ros::NodeHandle nh;

    XbeeReceiver xbee(nh);
    while ( 1 ) {
        xbee.update();
    }
    return 0;
}
