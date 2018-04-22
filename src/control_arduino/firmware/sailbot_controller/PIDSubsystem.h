#ifndef PIDSUBSYSTEM_H
#define PIDSUBSYSTEM_H

#include <ros.h>
#include <std_msgs/Float64.h>
#include "PWM.h"
#include "PID.h"

class PIDSubsystem {
public:
    PIDSubsystem(const char* name, int analogSensorPin, int pwmPin, double kP, double kI, double kD, ros::NodeHandle* _nh);
    PIDSubsystem(const char* name, int _analogSensorPin, int pwmPin, double _adcOffset, double _adcConversion, double kP, double kI, double kD, ros::NodeHandle* _nh);

    void configSetpointUnits(double adcOffset, double adcConversion);

    void configGains(double kP, double kI, double kD);
    void setRawSetpoint(int setpoint);
    void setSetpoint(double setpoint);
    void setOpenLoop(double speed);

    void configSetpointLimits(double low, double high);

    double getActual();

    void update();

    void debug();
private:
    ros::NodeHandle* nh;
    int analogSensorPin;
    PWM pwm;
    PID pid;
    bool controlActive;
    double adcOffset;
    double adcConversion;

    void updatePTerm(const std_msgs::Float64& pTerm);
    void updateITerm(const std_msgs::Float64& iTerm);
    void updateDTerm(const std_msgs::Float64& dTerm);

    double highSetpointLimit;
    double lowSetpointLimit;


    double actualPosition;
    ros::Subscriber<std_msgs::Float64, PIDSubsystem>* pConfigSub;
    ros::Subscriber<std_msgs::Float64, PIDSubsystem>* iConfigSub;
    ros::Subscriber<std_msgs::Float64, PIDSubsystem>* dConfigSub;
    ros::Publisher* actualPub;
    ros::Publisher* errorPub;

    std_msgs::Float64 actualPositionMsg;
    std_msgs::Float64 errorPositionMsg;

};

#endif
