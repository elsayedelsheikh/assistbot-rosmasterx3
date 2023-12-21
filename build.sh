#!/bin/bash
set -e
ws_dir=$(pwd)

## Build the ROS Workspace
cd $ws_dir
catkin_make
echo "source $ws_dir/devel/setup.bash" >> ~/.bashrc
# echo "export GAZEBO_MODEL_PATH=$ws_dir/ThirdParty/pacakge_name/models" >> ~/.bashrc
