#include "IMU.h"
#include "config.h"

IMU::IMU(ros::NodeHandle* _nh) : nh(_nh), bno(55) {
    bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF);

    imu_msg.header.frame_id = "boat";
    if ( shouldUseROS ) {
        imuPub = new ros::Publisher("imu", &imu_msg);
        imuStatePub = new ros::Publisher("imu/state", &imu_state_msg);
        nh->advertise(*imuPub);
        nh->advertise(*imuStatePub);
    }
}

double IMU::getHeading() {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    return(euler.z());
}

void IMU::update() {
    if ( shouldUseROS ) {
        // Retrieve and publish system status information
        uint8_t system = 0;
        uint8_t gyro = 0;
        uint8_t accel = 0;
        uint8_t mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
        imu_state_msg.header.stamp = nh->now();
        imu_state_msg.system = system;
        imu_state_msg.gyro = gyro;
        imu_state_msg.accel = accel;
        imu_state_msg.mag = mag;

        imuStatePub->publish(&imu_state_msg);
       
        // Retrieve and publish sensor data
        imu::Quaternion imuQuat = bno.getQuat();
        imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
        imu::Vector<3> angularVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

        imu_msg.header.stamp = nh->now();

        imu_msg.orientation.x = imuQuat.x();
        imu_msg.orientation.y = imuQuat.y();
        imu_msg.orientation.z = imuQuat.z();
        imu_msg.orientation.w = imuQuat.w();

        imu_msg.angular_velocity.x = angularVel.x();
        imu_msg.angular_velocity.y = angularVel.y();
        imu_msg.angular_velocity.z = angularVel.z();

        imu_msg.linear_acceleration.x = linearAccel.x();
        imu_msg.linear_acceleration.y = linearAccel.y();
        imu_msg.linear_acceleration.z = linearAccel.z();

        imuPub->publish(&imu_msg);
    }
}



