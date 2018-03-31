#ifndef GPS_ODOM_H
#define GPS_ODOM_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

class GPSOdom {
    public:
        GPSOdom(ros::NodeHandle &nh);
        void updateImu(const sensor_msgs::Imu::ConstPtr& imu);
        void updateFix(const sensor_msgs::NavSatFix::ConstPtr& fix);
        void updateVel(const geometry_msgs::TwistStamped::ConstPtr& twist);
        void update();
    private:
        ros::Subscriber gpsSub;
        ros::Subscriber imuSub;
        ros::Subscriber velSub;

        ros::Publisher odomPub;
        tf2_ros::TransformBroadcaster transform_broadcaster;

        nav_msgs::Odometry odom_msg;

        geometry_msgs::TransformStamped odomTransform;
	geometry_msgs::TransformStamped utmTransform;

        std::string base_link_frame;
        std::string odom_frame;
        std::string utm_frame;

        bool originSet;
        double originX, originY;
};

#endif

