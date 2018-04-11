#include "Arduino.h"
#include "XbeeCommunicationManager.h"

XbeeCommunicationManager::XbeeCommunicationManager(ros::NodeHandle* _nh) : nh(_nh) {
    if ( shouldUseROS ) {
        trueWindSub = new ros::Subscriber<sailbot_sim::TrueWind, XbeeCommunicationManager>("trueWind",&XbeeCommunicationManager::trueWindCb, this);
        cmdHeadingSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_heading",&XbeeCommunicationManager::cmdHeadingCb, this);
	cmdSailSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_sail_angle",&XbeeCommunicationManager::cmdSailCb, this);
	cmdRudderSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_rudder_angle",&XbeeCommunicationManager::cmdRudderCb, this);
	boatStateSub = new ross::Subsciber<visualization::BoatState, XbeeCommunicationManager>("boat_state",&XbeeCommunicationManager::boatStateCb, this);
        goalSub = new ros::Subscriber<objective::Goal, XbeeCommunicationManager>("goal", &XbeeCommunicationManager::goalCb, this);
	velocitySub = new ros::Subscriber<geometry_msgs::TwistStamped, XbeeCommunicationManager>("gps/vel",&XbeeCommunicationManager::velocityCb, this);
        gpsSub = new ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>("gps/fix", &XbeeCommunicationManager::gpsCb, this);

        nh->subscribe(*trueWindSub);
	nh->subscribe(*cmdHeadingSub);
	nh->subscribe(*cmdSailSub);
	nh->subscribe(*cmdRudderSub);
	nh->subscribe(*boatStateSub);
        nh->subscribe(*goalSub);
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
void XbeeCommunicationManager::updateSailAngle(PIDSubsystem sailPID) {
    xbee_info.curr_sail_angle = sailPID.getActual();
}

//update current rudder angle
void XbeeCommunicationManager::updateRudderAngle(PIDSubsystem rudderPID) {
    xbee_info.curr_rudder_angle = rudderPID.getActual();
}

//update state
void XbeeCommunicationManager::updateState() {
    xbee_info.state[0] = false;
    xbee_info.state[1] = false;
    switch (currentState) {
	case MODE_DISABLED:
	    xbee_info.state[0] = true;
	    break;
	case MODE_AUTONOMOUS:
	    xbee_info.state[1] = true;
    }

    if (shouldUseRos)
	xbee_info.state[2] = true;
    else
	xbee_info.state[2] = false;
}

void XbeeCommunicationManager::boatStateCb(const visualization::BoatState& state) {
    xbee_info.state[3] = state.navigation;
    xbee_info.state[4] = state.longDistance;
    xbee_info.state[5] = state.search;
    xbee_info.state[6] = state.stationKeeping;
}

//update goal info
void XbeeCommunicationManager::goalCb(const objective::Goal& goal) {
    xbee_info.goal_type = goal.goalType;
    xbee_info.goal_point[0] = goal.goalPoint.x;
    xbee_info.goal_point[1] = goal.goalPoint.y;
    xbee_info.goal_direction = goal.goalDirection;
}

//update current heading
void XbeeCommunicationManager::updateHeading(IMU imu) {
    xbee_info.curr_heading = imu.getHeading();
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
void XbeeCommunicationManager::updateBattery(VoltageMonitor voltage) {
    xbee_info.battery_volt = voltage.getBattery();
}

//update buoy positions
void XbeeCommunicationManager::updateBuoy() {
    nh.getParam("buoy1",xbee_info.buoy_pos[1]);
    nh.getParam("buoy2",xbee_info.buoy_pos[2]);
    nh.getParam("buoy3",xbee_info.buoy_pos[3]);
    nh.getParam("buoy4",xbee_info.buoy_pos[4]);
}

//public method to write to xbee
void XbeeCommunicationManager::WriteToXbee(char* input) {
    if (str_count==0) {
        strcpy(message, input);
	str_count = strlen(input);
    }
    else {
	strcat(message, input);
	str_count += strlen(input);
    }
    strcat(message, "\n");
    str_count += 2;
}

void XbeeCommunicationManager::update(PIDSubsystem sailPID, PIDSubsystem rudderPID, IMU imu, VoltageMonitor voltage) {
    //update info from teensy
    this->updateSailAngle(sailPID);
    this->updateRudderAngle(rudderPID);
    this->updateState();
    this->updateHeading(imu);
    this->updateBattery(voltage);
    if(shouldUseROS)
	this->updateBuoy();

    //send serial package with data
    Serial1.write((byte*)&xbee_info, sizeof(serial_packet));

    if(str_count!=0) { //if there is a string package to send, send the package
	xbee_str.size = str_count;
	Serial1.write((byte*)&xbee_str, sizeof(serial_string));
	str_count = 0;
    }
}


