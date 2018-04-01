#include "Arduino.h"
#include "XbeeCommunicationManager.h"

XbeeCommunicationManager::XbeeCommunicationManager(ros::Nodehandle* _nh) {
    this->nh = _nh;
    if(nh!=NULL) {
	batterySub = new ros::Subscriber<std_msgs::Float32, XbeeCommunicationManager>("/battery", &XbeeCommunicationManager::batteryCb, this);
	trueWindSub = new ros::Subscriber<sensors::msg::TrueWind, XbeeCommunicationManager>("/trueWind",&XbeeCommunicationManager::trueWindCb, this);
	goalPointSub = new ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>("/goal", &XbeeCommunicationManager::goalPointCb, this);
	odomSub = new ros::Subscriber<nav_msgs::Odometry, XbeeCommunicationManager>("/odometry/filtered", &XbeeCommunicationManager::odomCb, this);
	gpsSub = new ros::Subscriber<sensor_msgs::NatSatFix, XbeeCommunicationManager>("/gps/fix", &XbeeCommunicationManager::gpsCb, this);

        nh.subscribe(batterySub);
	nh.subscribe(trueWindSub);
        nh.subscribe(goalPointSub);
        nh.subscribe(odomSub);
        nh.subscribe(gpsSub);
    }

    serial_startval = -1386103603;
    xbee_info.serial_start = serial_startval;
}

void XbeeCommunicationManager::batteryCb(const std_msgs::Int32& batt) {
    xbee_info.battery_volt = batt.data;
}

void XbeeCommunicationManager::trueWindCb(const sensors::msg::TrueWind& truewind) {
    xbee_info.true_wind_speed = truewind.speed;
    xbee_info.true_wind_dir = truewind.direction;
}

void XbeeCommunicationManager::goalCb(const geometry_msgs::PointStamped& goal) {
    xbee_info.goal[0] = goal.point.x;
    xbee_info.goal[1] = goal.point.y;
}

void XbeeCommunicationManager::odomCb(const nav_msgs::Odometry& odom) {
    float x = odom.twist.twist.linear.x;
    float y = odom.twist.twist.linear.y;
    xbee_info.odom_speed = (x^2 + y^2)^0.5;

    float roll, pitch, yaw;
    tf::Quaternion quater;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quater);
    tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
    xbee_info.odom_heading = yaw;
}

void XbeeCommunicationManager::gpsCb(const sensor_msgs::NatSatFix& gps) {
    xbee_info.gps[0] = gps.latitude;
    xbee_info.gps[1] = gps.longitude;
}

void XbeeCommunicationManager::updateXbeeSailAngle(int sail_angle) {
    xbee_info.sail_angle = sail_angle;
}

void XbeeCommunicationManager::updateXbeeRudderAngle(int rudder) {
    xbee_info.rudder_angle = rudder;
}

void update() {
    if(shouldUseROS)
	xbee_info.state = 1;
    else xbee_info.state = 0;
    Serial1.write((byte*)&xbee_info, sizeof(serial_packet));
}


