#include "BoatSim.h"

BoatSim::BoatSim(ros::NodeHandle& n) :
    nh(n),
    angleSetpointSubscriber(n.subscribe("cmd_heading", 100, &BoatSim::updateAngleSetpoint, this)),
    windVectorSubscriber(n.subscribe("true_wind", 100, &BoatSim::updateWindVector, this)),
    resetPose(n.advertiseService("sim_reset_pose", &BoatSim::resetPoseService, this)),
    odomPublisher(n.advertise<nav_msgs::Odometry>("odom", 50)),
    tfBroadcaster(),
    windVector(0,0,0),
    x(0), y(0), heading(0) {
}

double boundAngle(double theta) {        
    while ( theta > 360.0 )
        theta -= 360.0;
    while ( theta < 0 )
        theta += 360.0;
    return theta;
}

double boundAngle180(double theta) {        
    while ( theta > 180.0 )
        theta -= 180.0;
    while ( theta < 0 )
        theta += 180.0;
    return theta;
}


void BoatSim::updateAngleSetpoint(std_msgs::Float32 angle) {
    heading = boundAngle(angle.data)*(M_PI/180.0);
}

void BoatSim::updateWindVector(geometry_msgs::Vector3 newWind) {
    windVector.setX(newWind.x);
    windVector.setY(newWind.y);
}

bool BoatSim::resetPoseService(sailbot_sim::reset_pose::Request& req, sailbot_sim::reset_pose::Response& res) {
  x = y = heading = 0;
  return true;
}

void BoatSim::run() {
    ros::Rate rate(60);

    ros::Time lastTime = ros::Time::now();
    while ( nh.ok() ) {
        ros::Time now = ros::Time::now();
        double dt = (now-lastTime).toSec();
        lastTime = now;

        double angleBetweenWind = boundAngle180(fabs(heading-atan2(windVector.y(), windVector.x()))*(180.0/M_PI));

        double velocity = 0;
        if ( angleBetweenWind > 43 && angleBetweenWind < 151 ) {
            velocity = 0.3;
        }

        y += velocity * dt * sin(heading);
        x += velocity * dt * cos(heading);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(heading);

        // Create odom -> base_link tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = now;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        tfBroadcaster.sendTransform(odom_trans);

        // Publish odom topic
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = velocity;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = 0;

        odomPublisher.publish(odom);

        ros::spinOnce();

        rate.sleep();
    }
}
    
