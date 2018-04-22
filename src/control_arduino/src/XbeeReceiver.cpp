#include "XbeeReceiver.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/ioctl.h>

XbeeReceiver::XbeeReceiver(ros::NodeHandle* _nh) :
    nh(_nh),
    true_wind_pub(nh->advertise<sensors::TrueWind>("trueWind", 10)),
    cmd_heading_pub(nh->advertise<std_msgs::Float32>("cmd_heading", 10)),
    cmd_rudder_pub(nh->advertise<std_msgs::Int32>("cmd_rudder_angle", 10)),
    cmd_sail_pub(nh->advertise<std_msgs::Int32>("cmd_sail_angle", 10)),
    curr_rudder_pub(nh->advertise<std_msgs::Int32>("curr_rudder_angle", 10)),
    curr_sail_pub(nh->advertise<std_msgs::Int32>("curr_sail_angle", 10)),
    //state_pub(nh->advertise<visualization::BoatState>("state", 10)),
    goal_pub(nh->advertise<objective::Goal>("goal", 10)),
    curr_heading_pub(nh->advertise<std_msgs::Float32>("curr_heading", 10)),
    vel_pub(nh->advertise<std_msgs::Float32>("velocity", 10)),
    lat_pub(nh->advertise<std_msgs::Float64>("latitude", 10)),
    long_pub(nh->advertise<std_msgs::Float64>("longitude", 10)),
    volt_pub(nh->advertise<std_msgs::Float32>("battery_voltage", 10))
{
    sock = open("/dev/ttyACM0", O_RDWR);
    if ( sock == -1 ) {
        perror("open()");
        exit(1);
        return;
    }

    struct termios settings;
    // Set baud rate
    tcgetattr(sock, &settings);
    cfsetospeed(&settings, B9600);
    cfmakeraw(&settings);

    tcsetattr(sock, TCSANOW, &settings);
    tcflush(sock, TCIOFLUSH);

    usleep(20000);
    char a = 'B';
    write(sock, &a, 1);

    bufPos = 0;

    processedPacket = false;
}

bool XbeeReceiver::hasByte() {
    int bytes = 0;
    if ( ioctl(sock, FIONREAD, &bytes) == -1 )
        perror("ioctl()");
    return bytes != 0;
}

char XbeeReceiver::getByte() {
    char c;
    read(sock, &c, 1);
    return c;
}

void XbeeReceiver::update() {
    if ( hasByte() ) {
        uint8_t buf = getByte();

        startPktBuffer = (uint32_t)startPktBuffer >> 8;
        uint32_t temp = (uint32_t)buf << 24;
        startPktBuffer = startPktBuffer | temp;
        if ( startPktBuffer == -1386103603 ) {
            bufPos = 0;
            processedPacket = false;

            #ifdef SERIAL_DEBUG
            ROS_INFO("Received new serial packet");
            #endif
        }
        else if ( bufPos == sizeof(serial_packet) ) {
            if ( !processedPacket ) {
                processedPacket = true;
                handleSerialPacket();

                #ifdef SERIAL_DEBUG
                ROS_INFO("Packet done");
                #endif
            }
        }
        else {
            ((char*)&packet)[bufPos++] = buf;
        }
    }
}

void XbeeReceiver::handleSerialPacket() {
    if((packet.true_wind_dir != FLT_MAX) && (packet.true_wind_speed != FLT_MAX))
    {
        true_wind_msg.header.stamp = last_recvd;
        true_wind_msg.direction = packet.true_wind_dir;
        true_wind_msg.speed = packet.true_wind_speed;
        true_wind_pub.publish(true_wind_msg);
    }
    
    if(packet.cmd_heading != FLT_MAX)
    {
        cmd_heading_msg.data = packet.cmd_heading;
        cmd_heading_pub.publish(cmd_heading_msg);   
    }    
     
    if(packet.cmd_sail_angle != INT32_MAX)
    {        
        cmd_sail_msg.data = packet.cmd_sail_angle;
        cmd_rudder_pub.publish(cmd_sail_msg);
    }
    
    if(packet.cmd_rudder_angle != INT32_MAX)
    {
        cmd_rudder_msg.data = packet.cmd_rudder_angle;
        cmd_sail_pub.publish(cmd_rudder_msg);
    }   

    if(packet.curr_sail_angle != INT32_MAX)
    {       
        curr_sail_msg.data = packet.curr_sail_angle;
        curr_rudder_pub.publish(curr_sail_msg);
    }
    
    if(packet.curr_rudder_angle != INT32_MAX)
    {             
        curr_rudder_msg.data = packet.curr_rudder_angle;
        curr_sail_pub.publish(curr_rudder_msg);
    }
             
    /*state_msg.header.stamp = last_recvd;
    state_msg.disabled = packet.state[0];
    state_msg.autonomous = packet.state[1];
    state_msg.transmittingROS = packet.state[2];
    state_msg.navigation = packet.state[3];
    state_msg.longDistance = packet.state[4];
    state_msg.search = packet.state[5];
    state_msg.stationKeeping = packet.state[6];
    state_pub.publish(state_msg);*/

    if((packet.goal_type != INT32_MAX) && (packet.goal_point[0] != DBL_MAX) && (packet.goal_point[1] != DBL_MAX) && (packet.goal_direction != INT32_MAX))
    {          
        goal_msg.header.stamp = last_recvd;
        goal_msg.goalType = packet.goal_type;
        goal_msg.goalPoint.x = packet.goal_point[0];
        goal_msg.goalPoint.y = packet.goal_point[1];
        goal_msg.goalPoint.z = 0;
        goal_msg.goalDirection = packet.goal_direction;
        goal_pub.publish(goal_msg);
    }
    
    if(packet.curr_heading != FLT_MAX)
    {               
        curr_heading_msg.data = packet.curr_heading;
        curr_heading_pub.publish(curr_heading_msg);
    }
    
    if(packet.velocity != FLT_MAX)
    {           
        vel_msg.data = packet.velocity;
        vel_pub.publish(vel_msg);
    }

    if(packet.gps[0] != DBL_MAX)
    {          
        lat_msg.data = packet.gps[0];
        lat_pub.publish(lat_msg);
    }

    if(packet.gps[1] != DBL_MAX)
    {               
        long_msg.data = packet.gps[1];
        long_pub.publish(long_msg);
    }
    
    if(packet.battery_volt != FLT_MAX)
    {              
        volt_msg.data = packet.battery_volt;
        volt_pub.publish(volt_msg); 
    }

    if((packet.buoy_pos[0][0] != DBL_MAX) && (packet.buoy_pos[0][1] != DBL_MAX))
    {           
        buoy_msg_1.header.stamp = last_recvd;
        buoy_msg_1.point.x = packet.buoy_pos[0][0];
        buoy_msg_1.point.y = packet.buoy_pos[0][1];
        buoy_msg_1.point.z = 0;
        buoy_pub_1.publish(buoy_msg_1);
    }
    
    if((packet.buoy_pos[1][0] != DBL_MAX) && (packet.buoy_pos[1][1] != DBL_MAX))
    {       
        buoy_msg_2.header.stamp = last_recvd;
        buoy_msg_2.point.x = packet.buoy_pos[1][0];
        buoy_msg_2.point.y = packet.buoy_pos[1][1];
        buoy_msg_2.point.z = 0;
        buoy_pub_2.publish(buoy_msg_2);
    }

    if((packet.buoy_pos[2][0] != DBL_MAX) && (packet.buoy_pos[2][1] != DBL_MAX))
    {               
        buoy_msg_3.header.stamp = last_recvd;
        buoy_msg_3.point.x = packet.buoy_pos[2][0];
        buoy_msg_3.point.y = packet.buoy_pos[2][1];
        buoy_msg_3.point.z = 0;
        buoy_pub_3.publish(buoy_msg_3);
    }

    if((packet.buoy_pos[3][0] != DBL_MAX) && (packet.buoy_pos[3][1] != DBL_MAX))
    {           
        buoy_msg_4.header.stamp = last_recvd;
        buoy_msg_4.point.x = packet.buoy_pos[3][0];
        buoy_msg_4.point.y = packet.buoy_pos[3][1];
        buoy_msg_4.point.z = 0;
        buoy_pub_4.publish(buoy_msg_4);
    }
}

