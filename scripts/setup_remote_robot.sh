#!/bin/bash
# Robot PC Setup - Run this on the robot

echo "🤖 Setting up Robot for Network Control"

# ROS2 Network Configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Source ROS2
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "✅ Robot ready for network control!"
echo "� Now start motor driver: ros2 run ros2_roboclaw_driver motor_driver"
