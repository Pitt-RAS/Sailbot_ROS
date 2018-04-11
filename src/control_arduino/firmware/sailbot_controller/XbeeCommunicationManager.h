#ifndef XBEE_COMM_MGR_H
#define XBEE_COMM_MGR_H

#include "config.h"
#include "VoltageMonitor.h"
#include "PIDSubsystem.h"
#include "IMU.h"
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/Quaternion.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sailbot_sim/TrueWind.h>
#include <objective/Goal.h>
#include <string.h>

struct serial_packet {
    int32_t start;
    uint8_t size;
    float true_wind_speed;
    float true_wind_dir;
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
    double buoy_pos[4][2];
};

struct serial_string {
    uint32_t start;
    uint8_t size;
    char message[256];
};

class XbeeCommunicationManager {
public:
    XbeeCommunicationManager(ros::NodeHandle*);
    void update(PIDSubsystem, PIDSubsystem, Imu, VoltageMonitor);
    void WriteToXbee(char*);
private:
    ros::NodeHandle* nh;

    serial_packet xbee_info;
    serial_string xbee_str;
    
    int str_count;

    int32_t serial_startval;
    void trueWindCb(const sailbot_sim::TrueWind&); 
    void cmdHeadingCb(const std_msgs::Int32&);
    void cmdSailCb(const std_msgs::Int32&);
    void cmdRudderCb(const std_msgs::Int32&);
    void updateSailAngle(PIDSubsystem);
    void updateRudderAngle(PIDSubsystem);
    void updateState();
    void boatStateCb();
    void goalCb(const objective::Goal&);
    void updateHeading(IMU);
    void velocityCb(const geometry_msgs::TwistStamped&);
    void gpsCb(const sensor_msgs::NavSatFix&);
    void updateBattery(VoltageMonitor);
    void updateBuoy();

    ros::Subscriber<sailbot_sim::TrueWind, XbeeCommunicationManager>* trueWindSub;
    ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* cmdHeadingSub;
    ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* cmdRudderSub;
    ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* cmdSailSub;
    ros::Subscriber<visualization::BoatState, XbeeCommunicationManager>* boatStateSub;
    ros::Subscriber<objective::Goal, XbeeCommunicationManager>* goalSub;
    ros::Subscriber<geometry_msgs::TwistStamped, XbeeCommunicationManager>* velocitySub;
    ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>* gpsSub;
};

#endif

