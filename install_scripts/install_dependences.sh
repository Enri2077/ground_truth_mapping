#!/bin/bash

cd ~/w/ros2_ws/src/performance_modelling
./install_dependencies.sh  # Installs dependencies with apt and pip
./install_dev.sh  # Installs the performance_modelling_py python package by linking to the source

sudo apt install -y ros-eloquent-navigation2 ros-eloquent-nav2-bringup # nav2
sudo apt install -y ros-eloquent-gazebo-ros ros-eloquent-gazebo-plugins ros-eloquent-gazebo-ros-pkgs ros-eloquent-launch* # gazebo
sudo apt install -y python3-pip
pip3 install pyquaternion

