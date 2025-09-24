#!/bin/bash
# Control PC Setup - Run this on your demo PC

echo "🖥️ Setting up Control PC for Robot Demo"

# ROS2 Network Configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Source ROS2
source /opt/ros/jazzy/setup.bash

echo "✅ Control PC ready!"
echo "🎮 Control robot: python3 remote_control.py keyboard"

echo "🌐 Network settings configured:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY" 
echo "  ROBOT_IP: $ROBOT_IP"

# Check if ROS2 is installed
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2 found, sourcing..."
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/galactic/setup.bash 2>/dev/null
    echo "✅ Control PC ready!"
    echo ""
    echo "💡 Test connection with:"
    echo "   ros2 topic list"
    echo ""
    echo "💡 Control the lawnmower with:"
    echo "   python3 remote_control.py keyboard"
else
    echo "❌ ROS2 not found on this PC"
    echo "📦 Install ROS2 first or use the standalone version"
    echo ""
    echo "💡 Alternative: Use the standalone control script (no ROS2 needed)"
fi
