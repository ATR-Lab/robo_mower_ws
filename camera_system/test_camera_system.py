#!/usr/bin/env python3

import sys
import subprocess

def test_camera_system():
    print("Camera System Test")
    print("==================")
    
    # Test 1: Check for camera devices
    print("1. Checking for camera devices...")
    try:
        import os
        video_devices = [f for f in os.listdir('/dev/') if f.startswith('video')]
        if video_devices:
            print(f"   Found devices: {video_devices}")
        else:
            print("   No video devices found in /dev/")
    except Exception as e:
        print(f"   Error checking devices: {e}")
    
    # Test 2: Check Python dependencies
    print("\n2. Checking Python dependencies...")
    
    # Check OpenCV
    try:
        import cv2
        print(f"   ✓ OpenCV: {cv2.__version__}")
    except ImportError:
        print("   ✗ OpenCV not found")
        
    # Check NumPy
    try:
        import numpy as np
        print(f"   ✓ NumPy: {np.__version__}")
    except ImportError:
        print("   ✗ NumPy not found")
    
    # Test 3: Check ROS2 dependencies
    print("\n3. Checking ROS2 dependencies...")
    
    # Check if ROS2 is sourced
    ros_distro = os.environ.get('ROS_DISTRO', None)
    if ros_distro:
        print(f"   ✓ ROS2 distribution: {ros_distro}")
    else:
        print("   ✗ ROS2 not sourced (source /opt/ros/jazzy/setup.bash)")
    
    # Check for cv_bridge
    try:
        import cv_bridge
        print("   ✓ cv_bridge available")
    except ImportError:
        print("   ✗ cv_bridge not found (install ros-jazzy-cv-bridge)")
    
    # Check for sensor_msgs
    try:
        from sensor_msgs.msg import Image, CompressedImage
        print("   ✓ sensor_msgs available")
    except ImportError:
        print("   ✗ sensor_msgs not found")
    
    # Test 4: Test camera access
    print("\n4. Testing camera access...")
    try:
        import cv2
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"   ✓ Camera 0 working - captured frame: {frame.shape}")
            else:
                print("   ✗ Camera 0 opened but failed to capture frame")
            cap.release()
        else:
            print("   ✗ Cannot open camera 0")
    except Exception as e:
        print(f"   ✗ Error testing camera: {e}")
    
    print("\n5. Running setup recommendations...")
    print("   To fix issues, run: ./setup_camera.sh")
    print("   Then test with: python3 camera_publisher.py")

if __name__ == '__main__':
    test_camera_system()
