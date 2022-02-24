cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  cv_camera
  image_proc
)
catkin_package(
)