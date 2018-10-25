cmake_minimum_required(VERSION 2.8.6)

project(ropod_experiment_executor)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  roslint
  ropod_ros_msgs
)

roslint_python()
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    ropod_ros_msgs
)
