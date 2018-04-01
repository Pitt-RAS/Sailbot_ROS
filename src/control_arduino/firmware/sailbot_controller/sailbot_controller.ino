#include "RobotState.h"
#include "PIDSubsystem.h"
#include "TransmitterInterface.h"
#include "config.h"
//#include <AS5045.h>
#include <ros.h>
#include <std_msgs/Int32.h>

bool shouldUseROS = false;

RobotState currentState = MODE_DISABLED;

PIDSubsystem* sail;
PIDSubsystem* leftRudder;
PIDSubsystem* rightRudder;

TransmitterInterface tx;

ros::NodeHandle nh;

std_msgs::Int32 windSensorTick;
ros::Publisher windSensorTickPublisher("wind_sensor_tick", &windSensorTick);

std_msgs::Int32 relativeWindDirection;
ros::Publisher relativeWindDirectionPublisher("/relative_wind_direction/raw", &relativeWindDirection);

//AS5045 angleSensor(A2, A3, A4) ;

void setup() {
    if ( shouldUseROS ) {
        nh.initNode();

        nh.advertise(windSensorTickPublisher);
        nh.advertise(relativeWindDirectionPublisher);
    }
    
    sail = new PIDSubsystem("sail", SAIL_POT, SAIL_PWM, SAIL_P, SAIL_I, SAIL_D, &nh);
    leftRudder = new PIDSubsystem("leftRudder", RUDDER_LEFT_POT, RUDDER_LEFT_PWM, RUDDER_LEFT_P, RUDDER_LEFT_I, RUDDER_RIGHT_D, &nh);
    rightRudder = new PIDSubsystem("rightRudder", RUDDER_RIGHT_POT, RUDDER_RIGHT_PWM, RUDDER_RIGHT_P, RUDDER_RIGHT_I, RUDDER_RIGHT_D, &nh);
    pinMode(2, INPUT);
}

void alwaysPeriodic() {

}

void teleopInit() {
}

void teleopPeriodic() {

}

void autonomousInit() {

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
    else if ( tx.wantsEnable() && tx.wantsTeleop() && currentState != MODE_TELEOP ) {
        teleopInit();
        currentState = MODE_TELEOP;
    }
    else if ( tx.wantsEnable() && currentState != MODE_DISABLED ) {
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
}

