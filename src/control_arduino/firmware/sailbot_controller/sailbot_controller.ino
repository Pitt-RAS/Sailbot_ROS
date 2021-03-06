#include "RobotState.h"
#include "PIDSubsystem.h"
#include "TransmitterInterface.h"
#include "Windsensors.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/Int32.h>
#include "Rate.h"
#include "Watchdog.h"
#include "XbeeCommunicationManager.h"
#include "VoltageMonitor.h"
#include "AutonomousCommandBridge.h"
#include "IMU.h"

bool shouldUseROS = true;
bool heartbeatLEDState = true;

RobotState currentState = MODE_DISABLED;

PIDSubsystem* sail;
PIDSubsystem* leftRudder;
PIDSubsystem* rightRudder;

VoltageMonitor* voltageMonitor;

TransmitterInterface* tx;

Windsensors* windsensors;

ros::NodeHandle nh;

Rate loopRate(MAIN_LOOP_HZ);
Rate heartbeatLEDLimiter(HEARTBEAT_DISABLED_HZ);
Rate xbeeRate(XBEE_LOOP_HZ);
Rate debugInfoRate(DEBUG_LOOP_HZ);

XbeeCommunicationManager* xbee;

AutonomousCommandBridge *autoBridge;

IMU* bno055;

void setup() {
    if ( shouldUseROS ) {
        nh.initNode();
    }

    analogReadResolution(13);

    tx = new TransmitterInterface(&nh);

    windsensors = new Windsensors(&nh);

    sail = new PIDSubsystem("sail", SAIL_POT, SAIL_PWM, SAIL_P, SAIL_I, SAIL_D, &nh);
    leftRudder = new PIDSubsystem("leftRudder", RUDDER_LEFT_POT, RUDDER_LEFT_PWM, RUDDER_LEFT_P, RUDDER_LEFT_I, RUDDER_RIGHT_D, &nh);
    rightRudder = new PIDSubsystem("rightRudder", RUDDER_RIGHT_POT, RUDDER_RIGHT_PWM, RUDDER_RIGHT_P, RUDDER_RIGHT_I, RUDDER_RIGHT_D, &nh);

    xbee = new XbeeCommunicationManager(&nh);
    voltageMonitor = new VoltageMonitor(&nh);
    autoBridge = new AutonomousCommandBridge(&nh);

    bno055 = new IMU(&nh);

    leftRudder->configSetpointUnits(2300, 4000.0/90.0);
    rightRudder->configSetpointUnits(2996, 4000.0/90.0);

    sail->configSetpointUnits(0, 8000/70.0);
    sail->configSetpointLimits(1000, 7000);

    leftRudder->configLimit(0.4);
    rightRudder->configLimit(0.4);

    pinMode(HEARTBEAT_LED, OUTPUT);
    disabledInit();
}

void alwaysPeriodic() {
    if ( heartbeatLEDLimiter.needsRun() ) {
        digitalWrite(HEARTBEAT_LED, heartbeatLEDState ? HIGH : LOW);
        heartbeatLEDState = !heartbeatLEDState;
    }

    windsensors->update();
    tx->update();

    if ( xbeeRate.needsRun() )
        xbee->update(sail, leftRudder, NULL, NULL);

    if ( debugInfoRate.needsRun() ) {
        sail->debug();
        leftRudder->debug();
        rightRudder->debug();
    }

    voltageMonitor->update();

    bno055->update();
}

void teleopInit() {
    heartbeatLEDLimiter.setRate(HEARTBEAT_TELEOP_HZ);
}

void teleopPeriodic() {
    sail->setSetpoint(tx->getSailAngle());
    double ra = tx->getRudderAngle();
    double sa = tx->getSailAngle();

    leftRudder->setSetpoint(ra);
    rightRudder->setSetpoint(ra);
    sail->setSetpoint(sa);
}

void autonomousInit() {
    heartbeatLEDLimiter.setRate(HEARTBEAT_AUTO_HZ);
    autoBridge->reset();
}

void autonomousPeriodic() {
    autoBridge->update(leftRudder, rightRudder, sail);
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

    if ( currentState != MODE_DISABLED && tx->wantsEnable() ) {
        enabledInit();
    }

    if ( tx->wantsEnable() && tx->wantsAutonomous() && currentState != MODE_AUTONOMOUS ) {
        autonomousInit();
        currentState = MODE_AUTONOMOUS;
    }
    else if ( tx->wantsEnable() && !tx->wantsAutonomous() && currentState != MODE_TELEOP ) {
        teleopInit();
        currentState = MODE_TELEOP;
    }
    else if ( !tx->wantsEnable() && currentState != MODE_DISABLED ) {
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
