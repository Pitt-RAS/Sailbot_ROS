#include "XbeeReceiver.h"

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
    volt_pub(nh->advertise<std_msgs::Float32>("battery_voltage", 10)),
    buoy_pub_1(nh->advertise<geometry_msgs::PointStamped>("buoy/1", 10)),
    buoy_pub_2(nh->advertise<geometry_msgs::PointStamped>("buoy/2", 10)),
    buoy_pub_3(nh->advertise<geometry_msgs::PointStamped>("buoy/3", 10)),
    buoy_pub_4(nh->advertise<geometry_msgs::PointStamped>("buoy/4", 10))
{    
    std::string port_name; //gets port name and opens
    nh->param<std::string>("port", port_name, "/dev/ttyUSB0");
    FILE* f_ptr = fopen(port_name.c_str(), "r");

    nh->param("start_value", start_val, -1386103603); //gets start value
}

void XbeeReceiver::update()
{
    serial_packet* serial_recvd = (serial_packet*)XbeeReceiver::receive(); //gets packet
    string_packet* string_recvd;
    
    if(serial_recvd->size==0) //0 => serial packet, else => console message
    {
        XbeeReceiver::publish(serial_recvd); //publishes
    }
    else
    {
        string_recvd = (string_packet*)serial_recvd; //prints message to ros console
        ROS_INFO("%s", string_recvd->buffer);
    }
}

char* XbeeReceiver::receive()
{   
    char* buffer = (char*)malloc(sizeof(serial_packet));

    uint8_t size = sizeof(serial_packet);
    int bytes_read = 0;
    
    int32_t start_buf = 0;
    uint8_t byte_buf = 0;
    uint32_t temp = 0;
    
    while(bytes_read < size) //blocks until all bytes in struct are read
    {        
        fread(&byte_buf, sizeof(uint8_t), 1, f_ptr); //reads byte
        
        start_buf = (uint32_t)start_buf >> 8; //shift inserts byte into start buffer
        temp = (uint32_t)byte_buf << 24;
        start_buf = start_buf | temp;
 
        buffer[bytes_read] = byte_buf; //adds byte to struct
        
        if(bytes_read==0) //gets size
        {
            size = byte_buf;
            
            if(size==0)
                size = sizeof(serial_packet);
        }
        
        bytes_read++;
        
        if(start_buf==start_val) //restarts reading if start value is read (dropped packet)
        {
            bytes_read = 0;
        }
    }

    last_recvd = ros::Time::now(); //gets time recvd for timestamp
    
    return(buffer);
}

void XbeeReceiver::publish(serial_packet* packet_to_pub)
{
    if((packet_to_pub->true_wind_dir != FLT_MAX) && (packet_to_pub->true_wind_speed != FLT_MAX))
    {
        true_wind_msg.header.stamp = last_recvd;
        true_wind_msg.direction = packet_to_pub->true_wind_dir;
        true_wind_msg.speed = packet_to_pub->true_wind_speed;
        true_wind_pub.publish(true_wind_msg);
    }
    
    if(packet_to_pub->cmd_heading != FLT_MAX)
    {
        cmd_heading_msg.data = packet_to_pub->cmd_heading;
        cmd_heading_pub.publish(cmd_heading_msg);   
    }    
     
    if(packet_to_pub->cmd_sail_angle != INT32_MAX)
    {        
        cmd_sail_msg.data = packet_to_pub->cmd_sail_angle;
        cmd_rudder_pub.publish(cmd_sail_msg);
    }
    
    if(packet_to_pub->cmd_rudder_angle != INT32_MAX)
    {
        cmd_rudder_msg.data = packet_to_pub->cmd_rudder_angle;
        cmd_sail_pub.publish(cmd_rudder_msg);
    }   

    if(packet_to_pub->curr_sail_angle != INT32_MAX)
    {       
        curr_sail_msg.data = packet_to_pub->curr_sail_angle;
        curr_rudder_pub.publish(curr_sail_msg);
    }
    
    if(packet_to_pub->curr_rudder_angle != INT32_MAX)
    {             
        curr_rudder_msg.data = packet_to_pub->curr_rudder_angle;
        curr_sail_pub.publish(curr_rudder_msg);
    }
             
    /*state_msg.header.stamp = last_recvd;
    state_msg.disabled = packet_to_pub->state[0];
    state_msg.autonomous = packet_to_pub->state[1];
    state_msg.transmittingROS = packet_to_pub->state[2];
    state_msg.navigation = packet_to_pub->state[3];
    state_msg.longDistance = packet_to_pub->state[4];
    state_msg.search = packet_to_pub->state[5];
    state_msg.stationKeeping = packet_to_pub->state[6];
    state_pub.publish(state_msg);*/

    if((packet_to_pub->goal_type != INT32_MAX) && (packet_to_pub->goal_point[0] != DBL_MAX) && (packet_to_pub->goal_point[1] != DBL_MAX) && (packet_to_pub->goal_direction != INT32_MAX))
    {          
        goal_msg.header.stamp = last_recvd;
        goal_msg.goalType = packet_to_pub->goal_type;
        goal_msg.goalPoint.x = packet_to_pub->goal_point[0];
        goal_msg.goalPoint.y = packet_to_pub->goal_point[1];
        goal_msg.goalPoint.z = 0;
        goal_msg.goalDirection = packet_to_pub->goal_direction;
        goal_pub.publish(goal_msg);
    }
    
    if(packet_to_pub->curr_heading != FLT_MAX)
    {               
        curr_heading_msg.data = packet_to_pub->curr_heading;
        curr_heading_pub.publish(curr_heading_msg);
    }
    
    if(packet_to_pub->velocity != FLT_MAX)
    {           
        vel_msg.data = packet_to_pub->velocity;
        vel_pub.publish(vel_msg);
    }

    if(packet_to_pub->gps[0] != DBL_MAX)
    {          
        lat_msg.data = packet_to_pub->gps[0];
        lat_pub.publish(lat_msg);
    }

    if(packet_to_pub->gps[1] != DBL_MAX)
    {               
        long_msg.data = packet_to_pub->gps[1];
        long_pub.publish(long_msg);
    }
    
    if(packet_to_pub->battery_volt != FLT_MAX)
    {              
        volt_msg.data = packet_to_pub->battery_volt;
        volt_pub.publish(volt_msg); 
    }

    if((packet_to_pub->buoy_pos[0][0] != DBL_MAX) && (packet_to_pub->buoy_pos[0][1] != DBL_MAX))
    {           
        buoy_msg_1.header.stamp = last_recvd;
        buoy_msg_1.point.x = packet_to_pub->buoy_pos[0][0];
        buoy_msg_1.point.y = packet_to_pub->buoy_pos[0][1];
        buoy_msg_1.point.z = 0;
        buoy_pub_1.publish(buoy_msg_1);
    }
    
    if((packet_to_pub->buoy_pos[1][0] != DBL_MAX) && (packet_to_pub->buoy_pos[1][1] != DBL_MAX))
    {       
        buoy_msg_2.header.stamp = last_recvd;
        buoy_msg_2.point.x = packet_to_pub->buoy_pos[1][0];
        buoy_msg_2.point.y = packet_to_pub->buoy_pos[1][1];
        buoy_msg_2.point.z = 0;
        buoy_pub_2.publish(buoy_msg_2);
    }

    if((packet_to_pub->buoy_pos[2][0] != DBL_MAX) && (packet_to_pub->buoy_pos[2][1] != DBL_MAX))
    {               
        buoy_msg_3.header.stamp = last_recvd;
        buoy_msg_3.point.x = packet_to_pub->buoy_pos[2][0];
        buoy_msg_3.point.y = packet_to_pub->buoy_pos[2][1];
        buoy_msg_3.point.z = 0;
        buoy_pub_3.publish(buoy_msg_3);
    }

    if((packet_to_pub->buoy_pos[3][0] != DBL_MAX) && (packet_to_pub->buoy_pos[3][1] != DBL_MAX))
    {           
        buoy_msg_4.header.stamp = last_recvd;
        buoy_msg_4.point.x = packet_to_pub->buoy_pos[3][0];
        buoy_msg_4.point.y = packet_to_pub->buoy_pos[3][1];
        buoy_msg_4.point.z = 0;
        buoy_pub_4.publish(buoy_msg_4);
    }
}
