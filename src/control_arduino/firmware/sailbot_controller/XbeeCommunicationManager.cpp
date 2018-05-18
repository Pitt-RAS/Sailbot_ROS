#include "Arduino.h"
#include "XbeeCommunicationManager.h"

XbeeCommunicationManager::XbeeCommunicationManager(ros::NodeHandle* _nh) : nh(_nh) {
  XBEE_SERIALPORT.begin(XBEE_BAUD);
  if ( shouldUseROS ) {
    trueWindSub = new ros::Subscriber<sensors::TrueWind, XbeeCommunicationManager>("trueWind",&XbeeCommunicationManager::trueWindCb, this);
    cmdHeadingSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_heading",&XbeeCommunicationManager::cmdHeadingCb, this);
    cmdSailSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_sail_angle",&XbeeCommunicationManager::cmdSailCb, this);
    cmdRudderSub = new ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>("cmd_rudder_angle",&XbeeCommunicationManager::cmdRudderCb, this);
    //boatStateSub = new ros::Subsciber<visualization::BoatState, XbeeCommunicationManager>("boat_state",&XbeeCommunicationManager::boatStateCb, this);
    goalSub = new ros::Subscriber<objective::Goal, XbeeCommunicationManager>("goal", &XbeeCommunicationManager::goalCb, this);
    velocitySub = new ros::Subscriber<geometry_msgs::TwistStamped, XbeeCommunicationManager>("gps/vel",&XbeeCommunicationManager::velocityCb, this);
    gpsSub = new ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>("gps/fix", &XbeeCommunicationManager::gpsCb, this);
    buoy1Sub = new ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>("buoy/1", &XbeeCommunicationManager::buoy1Cb, this);
    buoy2Sub = new ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>("buoy/2", &XbeeCommunicationManager::buoy2Cb, this);
    buoy3Sub = new ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>("buoy/3", &XbeeCommunicationManager::buoy3Cb, this);
    buoy4Sub = new ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>("buoy/4", &XbeeCommunicationManager::buoy4Cb, this);    
    
    nh->subscribe(*trueWindSub);
    nh->subscribe(*cmdHeadingSub);
    nh->subscribe(*cmdSailSub);
    nh->subscribe(*cmdRudderSub);
    //nh->subscribe(*boatStateSub);
    nh->subscribe(*goalSub);
    nh->subscribe(*velocitySub);
    nh->subscribe(*gpsSub);
    nh->subscribe(*buoy1Sub);
    nh->subscribe(*buoy2Sub);
    nh->subscribe(*buoy3Sub);
    nh->subscribe(*buoy4Sub);
  }
 
  serial_to_send.start = XBEE_START_VAL;
  console_to_send.start = XBEE_START_VAL;
  serial_to_send.size = 0; //size 0 indicates serial packet (instead of string/console packet)

  //set default values to MAX or false
  serial_to_send.true_wind_speed   = FLT_MAX;
  serial_to_send.true_wind_dir     = FLT_MAX;
  serial_to_send.cmd_heading       = FLT_MAX;
  serial_to_send.cmd_sail_angle    = INT32_MAX;
  serial_to_send.cmd_rudder_angle  = INT32_MAX;
  serial_to_send.curr_sail_angle   = INT32_MAX;
  serial_to_send.curr_rudder_angle = INT32_MAX;
  serial_to_send.goal_type         = INT32_MAX;
  serial_to_send.goal_point[0]     = DBL_MAX;
  serial_to_send.goal_point[1]     = DBL_MAX;
  serial_to_send.goal_direction    = INT32_MAX;
  serial_to_send.curr_heading      = FLT_MAX;
  serial_to_send.velocity          = FLT_MAX;
  serial_to_send.gps[0]            = DBL_MAX;
  serial_to_send.gps[1]            = DBL_MAX;
  serial_to_send.battery_volt      = FLT_MAX;
  for(int i = 0; i < 7; i++)
    serial_to_send.state[i] = false;
  for(int i = 0; i < 4; i++)
    for(int j = 0; j < 2; j++)
      serial_to_send.buoy_pos[i][j] = DBL_MAX;

}

//update true wind speed and direction
void XbeeCommunicationManager::trueWindCb(const sensors::TrueWind& truewind) {
  serial_to_send.true_wind_speed = truewind.speed;
  serial_to_send.true_wind_dir = truewind.direction;
}

//update command heading
void XbeeCommunicationManager::cmdHeadingCb(const std_msgs::Int32& head) {
  serial_to_send.cmd_heading = head.data;
}

//update command sail angle
void XbeeCommunicationManager::cmdSailCb(const std_msgs::Int32& sail) {
  serial_to_send.cmd_sail_angle = sail.data;
}

//update command rudder angle
void XbeeCommunicationManager::cmdRudderCb(const std_msgs::Int32& rudder) {
  serial_to_send.cmd_rudder_angle = rudder.data;
}

//update current sail angle
void XbeeCommunicationManager::updateSailAngle(PIDSubsystem* sailPID) {
  serial_to_send.curr_sail_angle = sailPID->getActual();
}

//update current rudder angle
void XbeeCommunicationManager::updateRudderAngle(PIDSubsystem* rudderPID) {
  serial_to_send.curr_rudder_angle = rudderPID->getActual();
}

//update state
void XbeeCommunicationManager::updateState() {
  if ( currentState == MODE_DISABLED )
    serial_to_send.state[0] = true;
  else
    serial_to_send.state[0] = false;

  if ( currentState == MODE_AUTONOMOUS )
    serial_to_send.state[1] = true;
  else
    serial_to_send.state[1] = false;
  
  if ( shouldUseROS )
    serial_to_send.state[2] = true;
  else
    serial_to_send.state[2] = false;
}

/*void XbeeCommunicationManager::boatStateCb(const visualization::BoatState& state) {
  serial_to_send.state[3] = state.navigation;
  serial_to_send.state[4] = state.longDistance;
  serial_to_send.state[5] = state.search;
  serial_to_send.state[6] = state.stationKeeping;
}*/

//update goal info
void XbeeCommunicationManager::goalCb(const objective::Goal& goal) {
    serial_to_send.goal_type = goal.goalType;
    serial_to_send.goal_point[0] = goal.goalPoint.x;
    serial_to_send.goal_point[1] = goal.goalPoint.y;
    serial_to_send.goal_direction = goal.goalDirection;
}

//update current heading
void XbeeCommunicationManager::updateHeading(IMU* imu) {
    serial_to_send.curr_heading = imu->getHeading();
}

//update current velocity
void XbeeCommunicationManager::velocityCb(const geometry_msgs::TwistStamped& vel) {
    float x, y, velo;
    x = vel.twist.linear.x;
    y = vel.twist.linear.y;
    velo = pow((pow(x, 2) + pow(y, 2)), 0.5);
    serial_to_send.velocity = velo;
}

//update current latitude and longitude
void XbeeCommunicationManager::gpsCb(const sensor_msgs::NavSatFix& gps) {
    serial_to_send.gps[0] = gps.latitude;
    serial_to_send.gps[1] = gps.longitude;
}

//update battery voltage
void XbeeCommunicationManager::updateBattery(VoltageMonitor* voltage) {
    serial_to_send.battery_volt = voltage->getVoltage();
}

//update buoy positions
void XbeeCommunicationManager::buoy1Cb(const geometry_msgs::PointStamped& buoy1) {
    serial_to_send.buoy_pos[0][0] = buoy1.point.x;
    serial_to_send.buoy_pos[0][1] = buoy1.point.y;
}

void XbeeCommunicationManager::buoy2Cb(const geometry_msgs::PointStamped& buoy2) {
    serial_to_send.buoy_pos[1][0] = buoy2.point.x;
    serial_to_send.buoy_pos[1][1] = buoy2.point.y;
}

void XbeeCommunicationManager::buoy3Cb(const geometry_msgs::PointStamped& buoy3) {
    serial_to_send.buoy_pos[2][0] = buoy3.point.x;
    serial_to_send.buoy_pos[2][1] = buoy3.point.y;
}

void XbeeCommunicationManager::buoy4Cb(const geometry_msgs::PointStamped& buoy4) {
    serial_to_send.buoy_pos[3][0] = buoy4.point.x;
    serial_to_send.buoy_pos[3][1] = buoy4.point.y;
}

//public method to write to xbee
int XbeeCommunicationManager::sendConsole(String msg) {
    if(msg.length()>256)
        return 1;

    char msg_as_array[256] = {'\0'};
    msg.toCharArray(msg_as_array, 256);
      
    console_to_send.size = msg.length();
    memcpy(console_to_send.buffer, msg_as_array, sizeof(console_to_send.buffer));
    
    XBEE_SERIALPORT.write((byte*)&console_to_send, sizeof(string_packet));
    
    return 0;
}

void XbeeCommunicationManager::update(PIDSubsystem* sailPID, PIDSubsystem* rudderPID, IMU* imu, VoltageMonitor* voltage) {
  //update info from teensy
  this->updateSailAngle(sailPID);
  this->updateRudderAngle(rudderPID);
  this->updateState();
//  this->updateHeading(imu);
  this->updateBattery(voltage);
  
  //send serial package with data
  XBEE_SERIALPORT.write((byte*)&serial_to_send, sizeof(serial_packet));
}


