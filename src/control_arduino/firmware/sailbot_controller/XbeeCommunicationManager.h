#ifndef XBEE_COMM_MGR_H
#define XBEE_COMM_MGR_H

#include "config.h"
#include "VoltageMonitor.h"
#include "PIDSubsystem.h"
#include "IMU.h"
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensors/TrueWind.h>
#include <objective/Goal.h>
#include <string.h>
#include <float.h>
#include <stdint.h>

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
  int8_t state[7];
  int32_t goal_type;
  double goal_point[2];
  int32_t goal_direction;
  float curr_heading;
  float velocity;
  double gps[2];
  float battery_volt;
  double buoy_pos[4][2];
} __attribute__((packed));

struct string_packet {
  int32_t start;
  uint8_t size;
  char buffer[256];
};

class XbeeCommunicationManager {
public:
  XbeeCommunicationManager(ros::NodeHandle*);
  void update(PIDSubsystem*, PIDSubsystem*, IMU*, VoltageMonitor*);
  int sendConsole(String);
private:
  ros::NodeHandle* nh;

  serial_packet serial_to_send;
  string_packet console_to_send;

  void trueWindCb(const sensors::TrueWind&); 
  void cmdHeadingCb(const std_msgs::Int32&);
  void cmdSailCb(const std_msgs::Int32&);
  void cmdRudderCb(const std_msgs::Int32&);
  void updateSailAngle(PIDSubsystem*);
  void updateRudderAngle(PIDSubsystem*);
  void updateState();
  //void boatStateCb();
  void goalCb(const objective::Goal&);
  void updateHeading(IMU*);
  void velocityCb(const geometry_msgs::TwistStamped&);
  void gpsCb(const sensor_msgs::NavSatFix&);
  void updateBattery(VoltageMonitor*);
  void buoy1Cb(const geometry_msgs::PointStamped&);
  void buoy2Cb(const geometry_msgs::PointStamped&);
  void buoy3Cb(const geometry_msgs::PointStamped&);
  void buoy4Cb(const geometry_msgs::PointStamped&);

  ros::Subscriber<sensors::TrueWind, XbeeCommunicationManager>* trueWindSub;
  ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* cmdHeadingSub;
  ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* cmdRudderSub;
  ros::Subscriber<std_msgs::Int32, XbeeCommunicationManager>* cmdSailSub;
  //ros::Subscriber<visualization::BoatState, XbeeCommunicationManager>* boatStateSub;
  ros::Subscriber<objective::Goal, XbeeCommunicationManager>* goalSub;
  ros::Subscriber<geometry_msgs::TwistStamped, XbeeCommunicationManager>* velocitySub;
  ros::Subscriber<sensor_msgs::NavSatFix, XbeeCommunicationManager>* gpsSub;
  ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>* buoy1Sub;
  ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>* buoy2Sub;
  ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>* buoy3Sub;
  ros::Subscriber<geometry_msgs::PointStamped, XbeeCommunicationManager>* buoy4Sub;
};

#endif

