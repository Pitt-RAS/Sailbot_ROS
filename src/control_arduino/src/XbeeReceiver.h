#ifndef XBEE_RECEIVER_H
#define XBEE_RECEIVER_H

#include <stdint.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <sailbot_sim/TrueWind.h>

struct serial_packet {
    int32_t true_wind_speed;
    int32_t true_wind_dir;
    float cmd_heading;
    int32_t cmd_sail_angle;
    int32_t cmd_rudder_angle;
    int32_t curr_sail_angle;
    int32_t curr_rudder_angle;
    int32_t state;
    float goal[2];
    float curr_heading;
    float velocity;
    double gps[2];
    float battery_volt;
};

class XbeeReceiver
{
public:
    XbeeReceiver(ros::NodeHandle nh);
    void update();
private:
    ros::NodeHandle* nh;
    sailbot_sim::TrueWind true_wind_msg;
    std_msgs::Float32 cmd_heading_msg;
    std_msgs::Int32 cmd_sail_msg;
    std_msgs::Int32 cmd_rudder_msg;
    std_msgs::Int32 curr_sail_msg;
    std_msgs::Int32 curr_rudder_msg;
    std_msgs::Bool state_msg;
    geometry_msgs::PointStamped goal_msg;
    std_msgs::Float32 curr_heading_msg;
    std_msgs::Float32 vel_msg;
    std_msgs::Float64 lat_msg;
    std_msgs::Float64 long_msg;
    std_msgs::Float32 volt_msg;
    ros::Publisher* true_wind_pub;
    ros::Publisher* cmd_heading_pub;
    ros::Publisher* cmd_rudder_pub;
    ros::Publisher* cmd_sail_pub;
    ros::Publisher* curr_rudder_pub;
    ros::Publisher* curr_sail_pub;
    ros::Publisher* state_pub;
    ros::Publisher* goal_pub;
    ros::Publisher* curr_heading_pub;
    ros::Publisher* vel_pub;
    ros::Publisher* lat_pub;
    ros::Publisher* long_pub;
    ros::Publisher* volt_pub;
    FILE* f_ptr;
    int32_t start_val;
    void receive();
    void publish();
};

#endif
