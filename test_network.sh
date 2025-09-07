#!/bin/bash
# Test ROS2 network connectivity

echo "🧪 Testing ROS2 Network Setup"

# Set environment
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Source ROS2
source /opt/ros/jazzy/setup.bash

echo "🔍 Checking ROS2 topics..."
timeout 10 ros2 topic list

echo "🔍 Checking /cmd_vel topic..."
timeout 5 ros2 topic info /cmd_vel

echo "✅ Network test complete!"
