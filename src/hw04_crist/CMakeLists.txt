cmake_minimum_required(VERSION 3.0.2)
project(hw04_crist)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# Define the catkin_package
catkin_package()

# Specify additional locations of header files
include_directories( include  ${catkin_INCLUDE_DIRS} )
