#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

ros::NodeHandle nh;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

sensor_msgs::Imu imu_msg;
ros::Publisher imuPub("/imu", &imu_msg);

void setup() {  
  imu_msg.header.frame_id = "base_link";

  nh.initNode();
  nh.advertise(imuPub);
  /*
   * IMUPLUS mode only fuses the gyro and accel
   * I would start with this to get stuff running, then try 
   * OPERATION_MODE_NDOF - this will fuse the compass as well,
   * and give an absolute heading. You'll also want to look at
   * waiting for calibration to finish, since this is when it'll jump from
   * being a realitive heading to an absolute heading. See the getCalibration method
   */
  bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
}

void loop() {

  imu::Quaternion imuQuat = bno.getQuat();
  imu::Vector<3> linearAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> angularVel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  imu_msg.header.stamp = nh.now();
  
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
  
  imuPub.publish(&imu_msg);

  nh.spinOnce();

  delay(10);
}
