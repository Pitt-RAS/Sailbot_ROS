#ifndef XBEE_RECEIVER_H
#define XBEE_RECEIVER_H

#include <stdint.h>
#include <ros.h>

struct serial_packet {
    int32_t true_wind_speed;
    int32_t true_wind_dir;
    int32_t sail_angle;
    int32_t rudder_angle;
    int32_t state;
    float goal[2];
    float odom_heading;
    float odom_speed;
    float gps[2];
    float battery_volt;
};

class XbeeReceiver
{
public:
    XbeeReceiver(ros::NodeHandle nh);
    void update();
private:
    ros::NodeHandle* nh;
    FILE* f_ptr;
    int32_t start_val;
    void receive();
    void publish();
};

#endif
