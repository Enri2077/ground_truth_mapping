cmake_minimum_required(VERSION 2.8.3)
project(ground_truth_mapping)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
)

