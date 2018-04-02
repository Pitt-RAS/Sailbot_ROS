#ifndef WINDSENSORS_H
#define WINDSENSORS_H

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <AS5045.h>

class Windsensors {
public:
    Windsensors(ros::NodeHandle* _nh);
    void update();

    unsigned long windSensorLastTick = millis();
    unsigned long windSensorDt = 0;
    bool windSensorDtUpdated = false;
private:
    ros::NodeHandle* nh;

    AS5045 angleSensor;

    std_msgs::Int32 windSensorTick;
    ros::Publisher* windSensorTickPublisher;

    std_msgs::Int32 relativeWindDirection;
    ros::Publisher* relativeWindDirectionPublisher;
};

#endif

