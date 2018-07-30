cmake_minimum_required(VERSION 2.8.3)
project(rr_auto_dock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  fiducial_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rr_auto_dock
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
