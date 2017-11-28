cmake_minimum_required(VERSION 2.8.3)
project(sailbot_sim)

find_package(catkin REQUIRED rospy std_msgs tf COMPONENTS message_generation) 

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

add_service_files(
  FILES
  ResetPose.srv
)
generate_messages()

catkin_package()
