#ifndef CONFIG_H
#define CONFIG_H

#include "RobotState.h"

extern bool shouldUseROS;
extern RobotState currentState;

// PID Constants
#define RUDDER_LEFT_P -0.0015
#define RUDDER_LEFT_I 0
#define RUDDER_LEFT_D 0

#define RUDDER_RIGHT_P -0.0016
#define RUDDER_RIGHT_I 0
#define RUDDER_RIGHT_D 0

#define SAIL_P 0.002
#define SAIL_I 0
#define SAIL_D 0

// Pin Assignments
#define SAIL_PWM 7
#define RUDDER_LEFT_PWM 6
#define RUDDER_RIGHT_PWM 5

#define SAIL_POT 17
#define RUDDER_LEFT_POT 14
#define RUDDER_RIGHT_POT 16

#define BATTERY_VOLTAGE A1

#define WINDSENSOR_PWM 26

#define HEARTBEAT_LED 13
#define HEARTBEAT_DISABLED_HZ 1
#define HEARTBEAT_AUTO_HZ 10
#define HEARTBEAT_TELEOP_HZ 5

#define WIND_SPEED_INT 2

// R9 transmitter
// Timeout before we consider the transmitter disconnected
#define TX_TIMEOUT 2000
#define TX_SERIALPORT Serial5

// XBee
#define XBEE_SERIALPORT Serial1
#define XBEE_BAUD 115200
#define SERIAL_START_VAL -1386103603

// Loop rates
#define MAIN_LOOP_HZ 100
#define XBEE_LOOP_HZ 3
#define DEBUG_LOOP_HZ 5

#endif

