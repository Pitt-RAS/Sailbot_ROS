#ifndef CONFIG_H
#define CONFIG_H

#include "RobotState.h"

extern bool shouldUseROS;
extern RobotState currentState;

// PID Constants
#define RUDDER_LEFT_P 0
#define RUDDER_LEFT_I 0
#define RUDDER_LEFT_D 0

#define RUDDER_RIGHT_P 0
#define RUDDER_RIGHT_I 0
#define RUDDER_RIGHT_D 0

#define SAIL_P 0
#define SAIL_I 0
#define SAIL_D 0

// Pin Assignments
#define SAIL_PWM 5
#define RUDDER_LEFT_PWM 6
#define RUDDER_RIGHT_PWM 7

#define SAIL_POT 14
#define RUDDER_LEFT_POT 15
#define RUDDER_RIGHT_POT 16

#define BATTERY_VOLTAGE A1

#endif
