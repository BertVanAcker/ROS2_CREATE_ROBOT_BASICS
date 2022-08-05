#!/bin/bash
# A sample Bash script, by Ryan
echo Updating the ROS2 node...

#fetch the latestet github push
git pull
#return to ROS2 base workspace
cd ~/dev_ws/
#build selected package
colcon build -- packages-select create_robot_movement_py
#source the installation
source install/setup.bash
