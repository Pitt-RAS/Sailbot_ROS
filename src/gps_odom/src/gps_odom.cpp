#include "gps_odom.h"
#include "conversions.h"

GPSOdom::GPSOdom(ros::NodeHandle& nh) : 
    originSet(false),
    gpsSub(nh.subscribe<sensor_msgs::NavSatFix>("fix", 10, &GPSOdom::updateFix, this)),
    imuSub(nh.subscribe<sensor_msgs::Imu>("imu", 10, &GPSOdom::updateImu, this)),
    odomPub(nh.advertise<nav_msgs::Odometry>("odometry/gps/raw", 10)) {
}

void GPSOdom::updateFix(const sensor_msgs::NavSatFix::ConstPtr& fix) {
    double northing, easting;
    std::string zone;
    gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

    if ( !originSet ) {
        originX = easting;
        originY = northing;
        originSet = true;
        ROS_WARN("GPSOdom set origin - received first GPS message");
    }

    odom_msg.pose.pose.position.x = easting-originX;
    odom_msg.pose.pose.position.y = northing-originY;
}

void GPSOdom::updateImu(const sensor_msgs::Imu::ConstPtr& imu) {
    odom_msg.pose.pose.orientation = imu->orientation;
}

void GPSOdom::update() {
    odomPub.publish(odom_msg);
}

