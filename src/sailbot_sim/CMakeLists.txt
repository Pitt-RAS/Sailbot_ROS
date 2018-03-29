cmake_minimum_required(VERSION 2.8.3)
project(sailbot_sim)

find_package(catkin REQUIRED rospy std_msgs tf COMPONENTS message_generation) 

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

add_service_files(
  FILES
  ResetPose.srv
)

add_message_files(
    FILES
    TrueWind.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_simulator.test)
endif()
