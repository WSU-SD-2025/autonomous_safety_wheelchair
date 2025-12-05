# Autonomous Safety Wheelchair

## Project Goals
- Bridge robotics with real human needs, providing indoor and outdoor mobility
- Enhance mobility and safety
- Real-time perception and navigation
- Reduce caregiver workload
- Increase user independence


## Sensors
- LiDAR
- RGB-D camera
- UWB
- IMU
- GPS

## Software Stack
- ROS2 (humble)
- Nav2

## Challenges
- Stability with rider
- Complex navigation environments
- Sourcing hardware

## Key Components
| Stage | Main Components | Key Topics | Purpose
|-------|-----------------|------------|--------
| Sensing | LiDAR, IMU, Wheel Odom, GPS | /scan, /imu, /odom, /gps | Collect environment and motion data.
| Localization | EKF1 + EKF2 + NavSat | /odometry/filtered, /odometry/gps | Use sensors to estimate wheelchair pose.
| Navigation | Nav2 Stack (Planner, Controller, Behavior Tree) | /plan, /plan_smoothed, /cmd_vel_nav | Plan and follow path to destination.
| Safety & Control | Velocity Smoother, Collision Monitor, Twist Mux | /cmd_vel_smooth, /cmd_vel_safe, /cmd_vel, (cmd_vel_keyboard) | Smooth and filter velocity to ensure safety.
| Motor Command | Motor Drive | /cmd_vel | Final velocity command to move wheelchair

## Nav2 Workflow
- Behavior Server
    + Planner Server
    + Smoother Server
    + Controller Server
    + Velocity Smoother
    + Collision Monitor
    + Twist Mux
    + Motor Driver

## Operating Modes
- Follow Mode
    + Wheelchair follows behind a guardian
- Lead-Ahead Mode
    + Wheelchair keeps a location ahead of the guardian
- Navigation Mode
    + Wheelchair moves to a defined location
- Manual Mode
    + Wheelchair moves according to user control inputs