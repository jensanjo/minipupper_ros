cmake_minimum_required(VERSION 3.0.2)
project(display_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)

catkin_package()