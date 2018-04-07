#include "Arduino.h"
#include "XbeeCommunicationManager.h"

XbeeCommunicationManager::XbeeCommunicationManager(ros::NodeHandle* _nh) : nh(_nh) {
    if ( shouldUseROS ) {
        trueWindSub = new ros::Subscriber<sailbot_sim::TrueWind, XbeeCommunicationManager>("trueWind",&XbeeCommunicationManager::trueWindCb, this);
	cmdHeadingSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_heading",&XbeeCommunicationManager::cmdHeadingCb, this);
	cmdSailSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_sail_angle",&XbeeCommunicationManager::cmdSailCb, this);
	cmdRudderSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_rudder_angle",&XbeeCommunicationManager::cmdRudderCb, this);
        goalSub = new ros::Subscriber<objective::msg::Goal, XbeeCommunicationManager>("goal", &XbeeCommunicationManager::goalCb, this);
	headingSub = new ros::Subscriber<sensor_msgs::Imu, XbeeCommunicationManager>("imu/data",&XbeeCommunicationManager::headingCb, this);
	velocitySub = new ros::Subscriber<geometry_msgs::TwistStamped, XbeeCommunicationManager>("gps/vel",&XbeeCommunicationManager::velocityCb, this);
        gpsSub = new ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>("gps/fix", &XbeeCommunicationManager::gpsCb, this);

        nh->subscribe(*trueWindSub);
	nh->subscribe(*cmdHeadingSub);
	nh->subscribe(*cmdSailSub);
	nh->subscribe(*cmdRudderSub);
        nh->subscribe(*goalSub);
	nh->subscribe(*headingSub);
	nh->subscribe(*velocitySub);
        nh->subscribe(*gpsSub);
    }

    serial_packet xbee_info;
    serial_startval = -1386103603;

    xbee_info.start = serial_startval;
    xbee_info.size = sizeof(serial_packet);

    xbee_str.start = serial_startval;
    str_count = 0;
}

//update true wind speed and direction
void XbeeCommunicationManager::trueWindCb(const sailbot_sim::TrueWind& truewind) {
    xbee_info.true_wind_speed = truewind.speed;
    xbee_info.true_wind_dir = truewind.direction;
}

//update command heading
void XbeeCommunicationManager::cmdHeadingCb(const std_msgs::Int32& head) {
    xbee_info.cmd_heading = head.data;
}

//update command sail angle
void XbeeCommunicationManager::cmdSailCb(const std_msgs::Int32& sail) {
    xbee_info.cmd_sail_angle = sail.data;
}

//update command rudder angle
void XbeeCommunicationManager::cmdRudderCb(const std_msgs::Int32& rudder) {
    xbee_info.cmd_rudder_angle = rudder.data;
}

//update current sail angle
void XbeeCommunicationManager::updateSailAngle() {
    xbee_info.curr_sail_angle = PIDSubsystem.getActual();
}

//update current rudder angle
void XbeeCommunicationManager::updateRudderAngle() {
    xbee_info.curr_rudder_angle = PIDSubsystem.getActual();
}

//update state
void XbeeCommunicationManager::updateState() {
    if ( shouldUseROS )
        xbee_info.state = 1;
    else
        xbee_info.state = 0;
}

//update goal
void XbeeCommunicationManager::goalCb(const objective::msg::Goal& goal) {
    xbee_info.goal_type = goal.goalType;
    xbee_info.goal_point[0] = goal.goalPoint.x;
    xbee_info.goal_point[1] = goal.goalPoint.y;
    xbee_info.goal_direction = goal.goalDirection;
}

//update current heading
void XbeeCommunicationManager::headingCb(const sensor_msgs::Imu& imu) {
    float roll, pitch, yaw;
    tf::Quaternion quater;
    tf::quaternionMsgToTF(imu.orientation, quater);
    tf::Matrix3x3(quater).getRPY(roll, pitch, yaw);
    xbee_info.curr_heading = yaw;
}

//update current velocity
void XbeeCommunicationManager::velocityCb(const geometry_msgs::TwistStamped& vel) {
    float x, y, velo;
    x = vel.twist.linear.x;
    y = vel.twist.linear.y;
    velo = (x^2 + y^2)^0.5;
    xbee_info.velocity = velo;
}

//update current latitude and longitude
void XbeeCommunicationManager::gpsCb(const sensor_msgs::NavSatFix& gps) {
    xbee_info.gps[0] = gps.latitude;
    xbee_info.gps[1] = gps.longitude;
}

//update battery voltage
void XbeeCommunicationManager::updateBattery() {
    xbee_info.battery_volt = VoltageMonitor::getBattery();
}

void WriteToXbee(const std::string& input) {
    std::strcpy(message, input.c_str());
    str_count = input.length();
}

void XbeeCommunicationManager::update() {
    //update info from teensy
    this->updateSailAngle();
    this->updateRudderAngle();
    this->updateState();
    this->updateBattery();

    //send serial package with data
    Serial1.write((byte*)&xbee_info, sizeof(serial_packet));

    if(str_count!=0) { //if there is a string package to send, send the package
	xbee_str.size = str_count;
	Serial1.write((byte*)&xbee_str, sizeof(serial_string));
    }
}


