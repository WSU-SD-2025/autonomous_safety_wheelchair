# Autonomous Safety Wheelchair - Setup Guide (Ubuntu 22.04 LTS, ROS2 Humble, Ignition Gazebo)
1. Make sure your linux version is Ubuntu 22.04 LTS
    - You can ask me Ubuntu 22.04 USB <br>
2. Install ROS2 humble version
    - Official installation guide is shared in Teams <br>
3. Install Ignition Gazebo 6.17.0
```
    #Ignition Gazebo Fortress 6.17.0
    sudo apt install -y ignition-fortress

    #Teleop Package
    sudo apt install -y ros-humble-teleop-twist-keyboard

    sudo apt install -y python3-colcon-common-extensions
```





## ROS-Gazebo Bridge Installation
```
sudo apt update
sudo apt install -y ros-humble-ros-gz ros-humble-ros-gz-bridge || true
```

## Git Clone & Workspace Build
```
    git clone <repo_url>
    cd ~/autonomous_safety_wheelchair
    sudo rosdep init || true
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install
```


## .bashrc setting
- Put these into your .bashrc commands
```
    #ROS2 Humble
    source /opt/ros/humble/setup.bash

    # ROS2 Senior Design workspace
    source ~/autonomous_safety_wheelchair/install/setup.bash

    # ROS2 Senior Design Gazebo
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix wheelchair_description)/share

    # ROS2 Sensor Path
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH

    # Short key for ROS2 keyboard-teleop
    alias kt='script -q /dev/null -c "LANG=C LC_ALL=C ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false -p repeat_rate:=10.0 -p key_timeout:=0.6"'
```


### Execution Command
1. Open Ignition gazebo with empty world & Spawn Wheelchair
```
ros2 launch wheelchair_description spawn_gazebo.launch.py
```

2. Drive wheelchair with keyboard - kt is shortcut that you set up in .bashrc file
```
kt
```
