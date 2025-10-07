#!/bin/bash

# This script adds necessary paths for launching the spawn_gazebo.launch.py file.
# Run this script before running
# 'ros2 launch wheelchair_description spawn_gazebo.launch.py'
#
# IMPORTANT NOTES:
#
#   - run this script using the command 'source ./env_setup.sh' or the
#     environment variables will only exist in the script's subprocess.
#
#   - run this script from your ros2 workspace (the 'autonomus_safety_wheelchair'
#     folder). This is required to make the command 'install/setup.bash' get the
#     correct environment.

source install/setup.bash

export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix wheelchair_description)/share;
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(ros2 pkg prefix wheelchair_description)/share;
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH;

# Feedback for the user to confirm it worked.
echo "Environment variables set:";
echo "GZ_SIM_RESOURCE_PATH = $GZ_SIM_RESOURCE_PATH";
echo "IGN_GAZEBO_RESOURCE_PATH = $IGN_GAZEBO_RESOURCE_PATH";
echo "IGN_GAZEBO_SYSTEM_PLUGIN_PATH = $IGN_GAZEBO_SYSTEM_PLUGIN_PATH";