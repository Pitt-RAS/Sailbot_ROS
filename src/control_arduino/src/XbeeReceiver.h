#ifndef XBEE_RECEIVER_H
#define XBEE_RECEIVER_H

#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PointStamped.h>
#include <objective/Goal.h>
#include <sailbot_sim/TrueWind.h>
#include <visualization/BoatState.h>

struct string_packet {
    uint8_t size;
    char buffer[256];
};

struct serial_packet {
    uint8_t size;
    int32_t true_wind_speed;
    int32_t true_wind_dir;
    float cmd_heading;
    int32_t cmd_sail_angle;
    int32_t cmd_rudder_angle;
    int32_t curr_sail_angle;
    int32_t curr_rudder_angle;
    bool state[7];
    int32_t goal_type;
    double goal_point[2];
    int32_t goal_direction;
    float curr_heading;
    float velocity;
    double gps[2];
    float battery_volt;
    double buoy_pos[2][4];
};

class XbeeReceiver
{
public:
    XbeeReceiver(ros::NodeHandle* nh);
    void update();
private:
    ros::NodeHandle* nh;
    sailbot_sim::TrueWind true_wind_msg;
    std_msgs::Float32 cmd_heading_msg;
    std_msgs::Int32 cmd_sail_msg;
    std_msgs::Int32 cmd_rudder_msg;
    std_msgs::Int32 curr_sail_msg;
    std_msgs::Int32 curr_rudder_msg;
    visualization::BoatState state_msg;
    objective::Goal goal_msg;
    std_msgs::Float32 curr_heading_msg;
    std_msgs::Float32 vel_msg;
    std_msgs::Float64 lat_msg;
    std_msgs::Float64 long_msg;
    std_msgs::Float32 volt_msg;
    geometry_msgs::PointStamped buoy_msg_1;
    geometry_msgs::PointStamped buoy_msg_2;
    geometry_msgs::PointStamped buoy_msg_3;
    geometry_msgs::PointStamped buoy_msg_4;
    ros::Publisher true_wind_pub;
    ros::Publisher cmd_heading_pub;
    ros::Publisher cmd_rudder_pub;
    ros::Publisher cmd_sail_pub;
    ros::Publisher curr_rudder_pub;
    ros::Publisher curr_sail_pub;
    ros::Publisher state_pub;
    ros::Publisher goal_pub;
    ros::Publisher curr_heading_pub;
    ros::Publisher vel_pub;
    ros::Publisher lat_pub;
    ros::Publisher long_pub;
    ros::Publisher volt_pub;
    ros::Publisher buoy_pub_1;
    ros::Publisher buoy_pub_2;
    ros::Publisher buoy_pub_3;
    ros::Publisher buoy_pub_4;
    
    FILE* f_ptr;
    int32_t start_val;
    ros::Time last_recvd;
    
    char* receive();
    void publish(serial_packet*);
};

#endif
