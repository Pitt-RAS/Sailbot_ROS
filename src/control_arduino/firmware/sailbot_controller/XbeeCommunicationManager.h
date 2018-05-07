#ifndef XBEE_COMM_MGR_H
#define XBEE_COMM_MGR_H

#include "config.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <sensors/TrueWind.h>

struct serial_packet {
    int32_t serial_start;
    int32_t true_wind_speed;
    int32_t true_wind_dir;
    int32_t sail_angle;
    int32_t rudder_angle;
    int32_t state;
    float goal[2];
    float odom_heading;
    float odom_speed;
    float gps[2];
    float battery_volt;
};

class XbeeCommunicationManager {
public:
    XbeeCommunicationManager(ros::NodeHandle* nh);
    void update();
    void updateXbeeWindData(int, int);
    void updateXbeeSailAngle(int);
    void updateXbeeRudderAngle(int);
private:
    ros::NodeHandle* nh;

    serial_packet xbee_info;

    int32_t serial_startval;
    void batteryCb(const std_msgs::Int32&);
    void trueWindCb(const sensors::TrueWind&);
    void goalPointCb(const geometry_msgs::PointStamped&);
    void gpsCb(const sensor_msgs::NavSatFix&);

    ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* batterySub;
    ros::Subscriber<sensors::TrueWind, XbeeCommunicationManager>* trueWindSub;
    ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>* goalPointSub;
    ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>* gpsSub;
};

#endif

