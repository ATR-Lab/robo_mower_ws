#!/bin/bash
# Daughter PC Setup - Run this on your current PC (ROS2 Humble)

echo "🖥️ Setting up Daughter PC for Robot Control"

# ROS2 Network Configuration
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Source ROS2 Humble (adjust if different path)
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -d "install" ]; then
    source install/setup.bash
fi

echo "✅ Daughter PC ready for network control!"
echo "🌐 Network settings configured:"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "  ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "  RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"

# Check network connectivity
echo ""
echo "🔍 Network diagnostics:"
echo "Local IP addresses:"
hostname -I

echo ""
echo "💡 Next steps:"
echo "1. Make sure Raspberry Pi is on same network"
echo "2. Build workspace: colcon build"
echo "3. Test connection: ros2 topic list"
echo "4. Run teleop: ros2 run robo_mower_teleop teleop_tank_robot"