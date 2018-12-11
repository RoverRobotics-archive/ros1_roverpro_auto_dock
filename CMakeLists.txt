cmake_minimum_required(VERSION 2.8.3)
project(rr_auto_dock)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  tf2
  fiducial_msgs
)

catkin_package()