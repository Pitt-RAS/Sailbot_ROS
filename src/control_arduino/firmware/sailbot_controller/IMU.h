#ifndef IMU_H
#define IMU_H

#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <Adafruit_BNO055.h>
#include <sensors/ImuState.h>

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
    sensors::ImuState imu_state_msg;
    ros::Publisher* imuStatePub;
    adafruit_bno055_offsets_t sensor_offsets;
    const int16_t ACCEL_X = -7;
    const int16_t ACCEL_Y = -30;
    const int16_t ACCEL_Z = 24;
    const int16_t MAG_X = 35;
    const int16_t MAG_Y = -112;
    const int16_t MAG_Z = -536;
    const int16_t GYRO_X = -2;
    const int16_t GYRO_Y = 0;
    const int16_t GYRO_Z = -1;
    const int16_t ACCEL_RADIUS = 1000;
    const int16_t MAG_RADIUS = 896;
};

#endif
