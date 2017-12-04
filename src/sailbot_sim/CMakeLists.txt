cmake_minimum_required(VERSION 2.8.3)
project(sailbot_sim)

find_package(catkin REQUIRED roscpp std_msgs tf COMPONENTS message_generation) 

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

add_service_files(
  FILES
  reset_pose.srv
)
generate_messages()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(sailbot_sim src/sailbot_sim.cpp src/BoatSim.cpp)
add_dependencies(sailbot_sim sailbot_sim_generate_messages_cpp)


target_link_libraries(sailbot_sim
  ${catkin_LIBRARIES}
)

