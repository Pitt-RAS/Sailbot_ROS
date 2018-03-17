import rospy
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void GPSOdomROSModule.GPSOdomCallback(msg)

	//Asynchronous module with only one callback!
	if(!run()):
    		return;

	GPS_pose.x = msg.pose.pose.position.y;
	GPS_pose.y = msg.pose.pose.position.x;
	GPS_pose.z = msg.pose.pose.position.z;

	ConvertToLocal(GPS_pose);

	publishGPSValue();
	return;

bool GPSOdomROSModule.GlobalToLocal(droneMsgsROS.dronePose *current)

	static droneMsgsROS::dronePose map_origin;
	static bool first_pose_received = false;

	bool initial_pose = !first_pose_received;

	if(initial_pose):
  	map_origin   = *current;
  	map_origin.x = map_origin.x;          	
  	map_origin.y = map_origin.y;         	
  	map_origin.z = map_origin.z;

  	first_pose_received = true;

  	ROS_INFO("INITIAL data (%.3f, %.3f, %.3f), map origin (%.3f, %.3f, %.3f)",
  	current.x, current.y, current.z,
  	map_origin.x, map_origin.y, map_origin.z);
	}

	current->x -= map_origin.x;
	current->y -= map_origin.y;
	current->z -= map_origin.z;
	
	return initial_pose;
}

int main(int argc, char** argv):
	rospy.init_node("objective_navigation")
	self.odom_sub = rospy.Subscriber("odometry/filtered",Odometry, 	self.localization_callback)
	buoy1 = rospy.get_param("buoyLocation1", 0)

	self.goalPublisher = rospy.Publisher("goal_point", Point, queue_size=10)

	double x = 0.0;
	double y = 0.0;

	float x_drone,y_drone;

 	GlobalFrame(0,0) = x_gps;
 	GlobalFrame(1,0) = y_gps;
 	GlobalFrame(2,0) = z_gps;
	
 	BodyFrame = RotationMat*GlobalFrame;
	
 	x_drone = BodyFrame(0,0);
 	y_drone = BodyFrame(1,0);

	int currentGoal=0;
	int distance;
	int point;

	currentGoal = (x_gps, y_gps);
	point1 = buoys1
	distance = ;
	while (currentGoal < len(Point)):
		if distance > 1:
	goal = new 
	self.goalPublisher.publish(goal)

