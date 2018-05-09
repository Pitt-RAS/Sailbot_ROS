#ifndef AUTONOMOUS_COMMAND_BRIDGE_H
#define AUTONOMOUS_COMMAND_BRIDGE_H

#include <ros.h>
#include <std_msgs/Int32.h>
#include "PIDSubsystem.h"

class AutonomousCommandBridge {
public:
    AutonomousCommandBridge(ros::NodeHandle* _nh);
    void reset();
    void update(PIDSubsystem* leftRudder, PIDSubsystem* rightRudder, PIDSubsystem* sail);
private:
    ros::NodeHandle* nh;
    ros::Subscriber<std_msgs::Int32, AutonomousCommandBridge>* cmdRudderAngleSub;
    ros::Subscriber<std_msgs::Int32, AutonomousCommandBridge>* cmdSailAngleSub;

    void updateCmdRudder(const std_msgs::Int32& cmd_angle);
    void updateCmdSail(const std_msgs::Int32& cmd_angle);

    bool sailGoalSet;
    bool rudderGoalSet;
    int sailGoal;
    int rudderGoal;
};

#endif

