#include <ros.h>
#include <std_msgs/Int32.h>

int updateWindVelocitySensor() {
  static unsigned long lastTime = millis();
  static unsigned long lastDuration = 0;

  static unsigned long lastDt = 0;
  
  int duration = pulseIn(2, HIGH);
  int delta = duration - lastDuration;
  if ( delta == 0 )
    lastDt = 0;

  lastDuration = duration;
  if ( abs(delta) > 100 && (millis()-lastTime) > 50 ) {
    unsigned long dt = millis() - lastTime;
    
    if ( delta < 0 )
      dt = -dt;

    lastTime = millis();
    lastDt = dt;
  }
  return lastDt;
}

ros::NodeHandle nh;

std_msgs::Int32 windSensorTickDt;
ros::Publisher windSensorTickDtPublisher("wind_sensor_tick_dt", &windSensorTickDt);

std_msgs::Int32 relativeWindDirection;
ros::Publisher relativeWindDirectionPublisher("/relative_wind_direction/raw", &relativeWindDirection);

void setup() {
  pinMode(2, INPUT);
  nh.initNode();
  
  nh.advertise(windSensorTickDtPublisher);
  nh.advertise(relativeWindDirectionPublisher);
}

void loop() {
  windSensorTickDt.data = updateWindVelocitySensor();
  windSensorTickDtPublisher.publish(&windSensorTickDt);

  relativeWindDirection.data = map(analogRead(0)+60, 0, 1024, 0, 360) % 360;
  relativeWindDirectionPublisher.publish(&relativeWindDirection);
  nh.spinOnce();
  delay(5);
}


