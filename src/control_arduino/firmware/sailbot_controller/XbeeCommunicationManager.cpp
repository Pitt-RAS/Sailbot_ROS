#include "Arduino.h"
#include "XbeeCommunicationManager.h"

XbeeCommunicationManager::XbeeCommunicationManager(ros::NodeHandle* _nh) : nh(_nh) {
    if ( shouldUseROS ) {
        batterySub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("/battery", &XbeeCommunicationManager::batteryCb, this);
        trueWindSub = new ros::Subscriber<sensors::TrueWind, XbeeCommunicationManager>("/trueWind",&XbeeCommunicationManager::trueWindCb, this);
        goalPointSub = new ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>("/goal", &XbeeCommunicationManager::goalPointCb, this);
        gpsSub = new ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>("/gps/fix", &XbeeCommunicationManager::gpsCb, this);

        nh->subscribe(*batterySub);
        nh->subscribe(*trueWindSub);
        nh->subscribe(*goalPointSub);
        nh->subscribe(*gpsSub);
    }

    serial_packet xbee_info;
    serial_startval = -1386103603;
    xbee_info.serial_start = serial_startval;
}

void XbeeCommunicationManager::batteryCb(const std_msgs::Int32& batt) {
    xbee_info.battery_volt = batt.data;
}

void XbeeCommunicationManager::trueWindCb(const sensors::TrueWind& truewind) {
    xbee_info.true_wind_speed = truewind.speed;
    xbee_info.true_wind_dir = truewind.direction;
}

void XbeeCommunicationManager::goalPointCb(const geometry_msgs::PointStamped& goal) {
    xbee_info.goal[0] = goal.point.x;
    xbee_info.goal[1] = goal.point.y;
}

void XbeeCommunicationManager::gpsCb(const sensor_msgs::NavSatFix& gps) {
    xbee_info.gps[0] = gps.latitude;
    xbee_info.gps[1] = gps.longitude;
}

void XbeeCommunicationManager::updateXbeeSailAngle(int sail_angle) {
    xbee_info.sail_angle = sail_angle;
}

void XbeeCommunicationManager::updateXbeeRudderAngle(int rudder) {
    xbee_info.rudder_angle = rudder;
}

void XbeeCommunicationManager::update() {
    if ( shouldUseROS )
        xbee_info.state = 1;
    else
        xbee_info.state = 0;
    Serial1.write((byte*)&xbee_info, sizeof(serial_packet));
}


