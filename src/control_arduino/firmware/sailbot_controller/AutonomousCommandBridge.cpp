#include "AutonomousCommandBridge.h"
#include "config.h"

AutonomousCommandBridge::AutonomousCommandBridge(ros::NodeHandle* _nh) :
    nh(_nh),
    cmdRudderAngleSub(new ros::Subscriber<std_msgs::Int32, AutonomousCommandBridge>("/cmd_rudder_angle", &AutonomousCommandBridge::updateCmdRudder, this)),
    cmdSailAngleSub(new ros::Subscriber<std_msgs::Int32, AutonomousCommandBridge>("/cmd_sail_angle", &AutonomousCommandBridge::updateCmdSail, this)),
    sailGoalSet(false),
    rudderGoalSet(false) {

    if ( shouldUseROS ) {
        nh->subscribe(*cmdRudderAngleSub);
        nh->subscribe(*cmdSailAngleSub);
    }
}

void AutonomousCommandBridge::reset() {
    sailGoalSet = false;
    rudderGoalSet = false;
}

void AutonomousCommandBridge::updateCmdRudder(const std_msgs::Int32& cmd_angle) {
    rudderGoal = cmd_angle.data;
    rudderGoalSet = true;
}

void AutonomousCommandBridge::updateCmdSail(const std_msgs::Int32& cmd_angle) {
    sailGoal = cmd_angle.data;
    sailGoalSet = true;
}

void AutonomousCommandBridge::update(PIDSubsystem* leftRudder, PIDSubsystem* rightRudder, PIDSubsystem* sail) {
    if ( sailGoalSet )
        sail->setSetpoint(sailGoal);

    if ( rudderGoalSet ) {
        rightRudder->setSetpoint(rudderGoal);
        leftRudder->setSetpoint(rudderGoal);
    }
}

