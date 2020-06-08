#!/bin/bash

mkdir -p ~/w/ros2_ws/src
cd ~/w/ros2_ws/src

git clone branch=eloquent-plugins-only https://github.com/Enri2077/gazebo_ros_pkgs.git
git clone https://github.com/Enri2077/ground_truth_mapping.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone https://github.com/Enri2077/performance_modelling.git

