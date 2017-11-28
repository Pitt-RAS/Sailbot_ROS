#ifndef BOATSIM_H
#define BOATSIM_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sailbot_sim/reset_pose.h> 

class BoatSim {
private:
    ros::NodeHandle& nh;
    ros::Subscriber angleSetpointSubscriber;
    ros::Subscriber windVectorSubscriber;
    ros::ServiceServer resetPose;

    ros::Publisher odomPublisher;

    tf::TransformBroadcaster tfBroadcaster;

    tf::Vector3 windVector;
    std_msgs::Float32 newTheta;

    double x,y,heading;

public:
    BoatSim(ros::NodeHandle& n);
    void updateSailAngle(std_msgs::Float32 angle); 
    void updateAngleSetpoint(std_msgs::Float32 newTheta);
    void updateWindVector(geometry_msgs::Vector3 newWind);
    bool resetPoseService(sailbot_sim::reset_pose::Request& req, sailbot_sim::reset_pose::Response& res); 
    void run();
};
#endif
