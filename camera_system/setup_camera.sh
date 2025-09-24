#!/bin/bash

echo "Setting up Camera Publisher/Subscriber System"
echo "=============================================="

# Update package list
echo "Updating package list..."
sudo apt update

# Install camera and video dependencies
echo "Installing camera dependencies..."
sudo apt install -y \
    v4l-utils \
    ffmpeg \
    python3-opencv \
    python3-pip

# Install ROS2 dependencies if not already installed
echo "Installing ROS2 Python dependencies..."
pip3 install \
    opencv-python \
    numpy

# Install ROS2 packages for camera support
echo "Installing ROS2 camera packages..."
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-transport-plugins \
    ros-jazzy-compressed-image-transport

# Check for available cameras
echo "Checking for available cameras..."
if ls /dev/video* 1> /dev/null 2>&1; then
    echo "Found cameras:"
    ls -la /dev/video*
    echo ""
    echo "Camera details:"
    v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl not available for camera details"
else
    echo "No cameras found in /dev/video*"
    echo "You may need to:"
    echo "1. Connect a USB camera"
    echo "2. Enable the camera module (for RPi)"
    echo "3. Check camera permissions"
fi

# Set permissions for video devices
echo "Setting up camera permissions..."
sudo usermod -a -G video $USER

echo ""
echo "Setup complete!"
echo "==============="
echo ""
echo "To test the camera system:"
echo "1. Logout and login (or reboot) to apply video group permissions"
echo "2. Run: python3 camera_publisher.py"
echo "3. In another terminal: python3 camera_subscriber.py"
echo ""
echo "For network access from remote computers, make sure:"
echo "- ROS_DOMAIN_ID is the same on both computers"
echo "- Network connectivity is working"
echo "- Firewall allows ROS2 communication"
echo ""
echo "See CAMERA_README.md for detailed usage instructions."
