#include "gps_odom.h"
#include "conversions.h"

GPSOdom::GPSOdom(ros::NodeHandle& nh) :
    originSet(false),
    gpsSub(nh.subscribe<sensor_msgs::NavSatFix>("fix", 10, &GPSOdom::updateFix, this)),
    imuSub(nh.subscribe<sensor_msgs::Imu>("imu", 10, &GPSOdom::updateImu, this)),
    velSub(nh.subscribe<geometry_msgs::TwistStamped>("vel", 10, &GPSOdom::updateVel, this)),
    odomPub(nh.advertise<nav_msgs::Odometry>("odometry/gps/raw", 10)),
    odom_frame("odom"),
    base_link_frame("boat"),
    utm_frame("utm") {

    transform.header.frame_id = odom_frame;
    transform.child_frame_id = base_link_frame;

    geometry_msgs::Quaternion zeroQuat;
    zeroQuat.x = zeroQuat.y = zeroQuat.z = 0;
    zeroQuat.w = 1;

    transform.transform.rotation = zeroQuat;
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

    geometry_msgs::Vector3 position;
    position.x = odom_msg.pose.pose.position.x;
    position.y = odom_msg.pose.pose.position.y;
    position.z = 0;
    transform.transform.translation = position;

    transform.header.stamp = ros::Time::now();
    transform_broadcaster.sendTransform(transform);
}

void GPSOdom::updateImu(const sensor_msgs::Imu::ConstPtr& imu) {
    odom_msg.pose.pose.orientation = imu->orientation;
    transform.transform.rotation = imu->orientation;
}

void GPSOdom::updateVel(const geometry_msgs::TwistStamped::ConstPtr& twist) {
    odom_msg.twist.twist = twist->twist;
}

void GPSOdom::update() {
    odomPub.publish(odom_msg);
}

