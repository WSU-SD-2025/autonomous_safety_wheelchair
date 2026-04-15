#!/bin/bash

# ONLY run this script if a ros2 docker container does not already exist. Use docker ps -a
# to check. A stopped container can be restarted.

# Notes:
# -it : starts the container in interactive mode, allowing you to interact with the terminal.
# --runtime nvidia : Use the NVIDIA runtime.
# --network host : Use the host's network stack.
# -v source:destination : Mounts a volume from the host to the container.

sudo docker run \
    -it \
    --runtime nvidia \
    --network host \
    -v /home/asw/Volumes/Ros2_Container_Volume:/volume \
    dustynv/ros:humble-ros-base-l4t-r36.3.0 \
    /bin/bash