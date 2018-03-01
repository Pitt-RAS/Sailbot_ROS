#ifndef GPS_ODOM_H
#define GPS_ODOM_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

class GPSOdom {
    public:
        GPSOdom(ros::NodeHandle &nh);
        void updateImu(const sensor_msgs::Imu::ConstPtr& imu);
        void updateFix(const sensor_msgs::NavSatFix::ConstPtr& fix);
        void update();
    private:
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Publisher odomPub;

        nav_msgs::Odometry odom_msg;
        bool originSet;
        double originX, originY;
};

#endif

