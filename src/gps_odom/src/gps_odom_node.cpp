#include <ros/ros.h>
#include "gps_odom.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "gps_odom");
    ros::NodeHandle nh;

    ros::Rate rate(10);

    GPSOdom gps_odom(nh);

    while ( nh.ok() ) {
        gps_odom.update();
        ros::spinOnce();
        rate.sleep();
    }
}
