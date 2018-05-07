#ifndef IMU_H
#define IMU_H

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_BNO055.h>

class IMU {
public:
    IMU(ros::NodeHandle* _nh);

    double getHeading();
    void update();
private:
    ros::NodeHandle* nh;
    Adafruit_BNO055 bno;
    sensor_msgs::Imu imu_msg;
    ros::Publisher* imuPub;
};

#endif
