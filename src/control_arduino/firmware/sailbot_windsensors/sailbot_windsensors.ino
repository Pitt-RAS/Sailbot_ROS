#include <AS5045.h>
#include <ros.h>
#include <std_msgs/Int32.h>


unsigned long windSensorLastTick = millis();
unsigned long windSensorDt = 0;
bool windSensorDtUpdated = false;
void windISR() {
  if ( windSensorDtUpdated )
    return;
  windSensorDt = millis() - windSensorLastTick;
  windSensorLastTick = millis();
  windSensorDtUpdated = true;
}

ros::NodeHandle nh;

std_msgs::Int32 windSensorTick;
ros::Publisher windSensorTickPublisher("wind_sensor_tick", &windSensorTick);

std_msgs::Int32 relativeWindDirection;
ros::Publisher relativeWindDirectionPublisher("/relative_wind_direction/raw", &relativeWindDirection);

AS5045 angleSensor(A2, A3, A4) ;

void setup() {
  angleSensor.begin();

  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), windISR, RISING);
  nh.initNode();

  nh.advertise(windSensorTickPublisher);
  nh.advertise(relativeWindDirectionPublisher);
}

void loop() {
  if ( windSensorDtUpdated ) {
    windSensorTick.data = windSensorDt;
    windSensorTickPublisher.publish(&windSensorTick);
    windSensorDtUpdated = false;
  }

  relativeWindDirection.data = angleSensor.read();
  relativeWindDirectionPublisher.publish(&relativeWindDirection);
  nh.spinOnce();
  delay(10);
}

