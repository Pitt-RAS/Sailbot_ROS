#include <stdlib.h>
#include <stdint.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <sailbot_sim/TrueWind.h>
#include "XbeeReceiver.h"

XbeeReceiver::XbeeReceiver(ros::NodeHandle _nh) : nh(_nh)
{
  nh.getParam("port", port) //gets port name and opens
  f_ptr = fopen(PORT, "r");
  
  nh.getParam("start_value", start_val) //gets start value
}

void XbeeReceiver::update() //receives and publishes from xbee
{
    serial_packet recvd_packet = receive();
    publish(recvd_packet);
}

serial_packet XbeeReceiver::receive()
{   
    serial_packet packet_to_recv;
    void* buffer = (void*)&packet_to_recv;
    
    int bytes_read = 0;
    
    int32_t start_buf = 0;
    uint8_t byte_buf = 0;
    uint32_t temp = 0;
    
    while(bytes_read!=sizeof(serial_packet)) //blocks until all bytes in struct are read
    {        
        fread(&byte_buf, sizeof(uint8_t), 1, f_ptr); //reads byte
        start_buf = (uint32_t)start_buf >> 8; //shift inserts byte into start buffer
        temp = (uint32_t)byte_buf << 24;
        start_buf = start_buf | temp;
        
        buffer[bytes_read] = byte_buf; //adds byte to struct
        
        bytes_read++;
        
        if(start_buf==start_val) //restarts reading if start value is read (dropped packet)
            bytes_read = 0;
    }
    
    return(packet_to_recv);
}

void XbeeReceiver::publish()
{
    /*TO DO: add ROS publishers*/
}
