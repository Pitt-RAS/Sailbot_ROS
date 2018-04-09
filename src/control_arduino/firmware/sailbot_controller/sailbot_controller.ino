#include "RobotState.h"
#include "PIDSubsystem.h"
#include "TransmitterInterface.h"
#include "Windsensors.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include "Rate.h"

bool shouldUseROS = true;
bool heartbeatLEDState = true;

RobotState currentState = MODE_DISABLED;

PIDSubsystem* sail;
PIDSubsystem* leftRudder;
PIDSubsystem* rightRudder;

TransmitterInterface tx;

Windsensors* windsensors;

ros::NodeHandle nh;

Rate loopRate(100);
Rate heartbeatLEDLimiter(HEARTBEAT_DISABLED_HZ);

void setup() {
    if ( shouldUseROS ) {
        nh.initNode();
    }

    windsensors = new Windsensors(&nh);

    sail = new PIDSubsystem("sail", SAIL_POT, SAIL_PWM, SAIL_P, SAIL_I, SAIL_D, &nh);
    leftRudder = new PIDSubsystem("leftRudder", RUDDER_LEFT_POT, RUDDER_LEFT_PWM, RUDDER_LEFT_P, RUDDER_LEFT_I, RUDDER_RIGHT_D, &nh);
    rightRudder = new PIDSubsystem("rightRudder", RUDDER_RIGHT_POT, RUDDER_RIGHT_PWM, RUDDER_RIGHT_P, RUDDER_RIGHT_I, RUDDER_RIGHT_D, &nh);

    pinMode(2, INPUT);
    pinMode(HEARTBEAT_LED, OUTPUT);
    disabledInit();
}

void alwaysPeriodic() {
    if ( heartbeatLEDLimiter.needsRun() ) {
        digitalWrite(HEARTBEAT_LED, heartbeatLEDState ? HIGH : LOW);
        heartbeatLEDState = !heartbeatLEDState;
    }

    windsensors->update();
    tx.update();
}

void teleopInit() {
    heartbeatLEDLimiter.setRate(HEARTBEAT_TELEOP_HZ);
}

void teleopPeriodic() {
    sail->setSetpoint(tx.getSailAngle());
    double ra = tx.getRudderAngle();
    leftRudder->setSetpoint(ra);
    rightRudder->setSetpoint(ra);
}

void autonomousInit() {
    heartbeatLEDLimiter.setRate(HEARTBEAT_AUTO_HZ);
}

void autonomousPeriodic() {

}

void enabledInit() {
}

void enabledPeriodic() {
    sail->update();
    leftRudder->update();
    rightRudder->update();
}

void disabledInit() {
    heartbeatLEDLimiter.setRate(HEARTBEAT_DISABLED_HZ);
}

void disabledPeriodic() {
}

void loop() {

    if ( currentState != MODE_DISABLED && tx.wantsEnable() ) {
        enabledInit();
    }

    if ( tx.wantsEnable() && tx.wantsAutonomous() && currentState != MODE_AUTONOMOUS ) {
        autonomousInit();
        currentState = MODE_AUTONOMOUS;
    }
    else if ( tx.wantsEnable() && !tx.wantsAutonomous() && currentState != MODE_TELEOP ) {
        teleopInit();
        currentState = MODE_TELEOP;
    }
    else if ( !tx.wantsEnable() && currentState != MODE_DISABLED ) {
        disabledInit();
        sail->setOpenLoop(0);
        leftRudder->setOpenLoop(0);
        rightRudder->setOpenLoop(0);
        currentState = MODE_DISABLED;
    }

    if ( currentState == MODE_DISABLED ) {
        disabledPeriodic();
    }
    else if ( currentState == MODE_AUTONOMOUS ) {
        autonomousPeriodic();
    }
    else if ( currentState == MODE_TELEOP ) {
        teleopPeriodic();
    }

    if ( currentState != MODE_DISABLED )
        enabledPeriodic();

    alwaysPeriodic();

    if ( shouldUseROS ) {
        nh.spinOnce();
    }

    loopRate.sleep();
}


