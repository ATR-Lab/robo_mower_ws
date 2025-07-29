#!/bin/bash

# Quick setup script for ROS2 RoboClaw Dual Motor Control System
# Run this after cloning the repository

echo "🚀 Setting up ROS2 RoboClaw Dual Motor Control System..."

# Check if ROS2 is installed
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please install ROS2 Jazzy first:"
    echo "   sudo apt update && sudo apt install ros-jazzy-desktop-full"
    exit 1
fi

# Install Python dependencies
echo "📦 Installing Python dependencies..."
pip3 install pyserial

# Build the workspace
echo "🔧 Building ROS2 workspace..."
cd ~/ros2_ws
colcon build

# Source the workspace
echo "⚙️  Sourcing workspace..."
source install/setup.bash

# Check for connected devices
echo "🔍 Checking for connected devices..."
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "✅ Found USB devices:"
    ls -la /dev/ttyACM*
else
    echo "⚠️  No /dev/ttyACM* devices found. Please connect your hardware."
fi

echo ""
echo "✅ Setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Connect your RoboClaw to USB (should appear as /dev/ttyACM0)"
echo "2. Connect your Arduino to USB (should appear as /dev/ttyACM1)" 
echo "3. Upload the Arduino sketch: arduino_roboclaw_usb_bridge/encoder_publisher.ino"
echo "4. See README.md for running the system"
echo ""
echo "📖 Full documentation: FINAL_SYSTEM_DOCUMENTATION.md"
