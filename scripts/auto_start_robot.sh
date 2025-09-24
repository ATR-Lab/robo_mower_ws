#!/bin/bash
# Auto-start script for Robot PC
# This will run automatically when robot boots up

echo "🤖 Starting Lawnmower Robot System..."

# Set environment
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Source ROS2
source /opt/ros/jazzy/setup.bash
source /home/alwinsdon/ros2_ws/install/setup.bash

cd /home/alwinsdon/ros2_ws

echo "🔧 Waiting for hardware to initialize..."
sleep 5

# Check USB devices
echo "🔍 Checking USB devices..."
lsusb | grep -E "(Arduino|Roboclaw|2341:0043|03eb:2404)"

# Wait for serial devices
echo "⏳ Waiting for serial devices..."
timeout=30
while [ $timeout -gt 0 ] && [ ! -e /dev/ttyACM0 ] && [ ! -e /dev/ttyACM1 ]; do
    sleep 1
    timeout=$((timeout-1))
done

if [ -e /dev/ttyACM0 ] || [ -e /dev/ttyACM1 ]; then
    echo "✅ Serial devices found:"
    ls -la /dev/ttyACM* 2>/dev/null
    
    echo "🚀 Starting motor driver..."
    ros2 run ros2_roboclaw_driver motor_driver &
    MOTOR_PID=$!
    
    sleep 3
    
    echo "🌐 Starting HTTP bridge..."
    python3 http_bridge.py &
    HTTP_PID=$!
    
    echo "✅ Robot system started!"
    echo "📱 Web interface: http://$(hostname -I | awk '{print $1}'):8080"
    echo "🎮 Ready for remote control!"
    
    # Keep running and monitor processes
    while true; do
        # Check if motor driver is still running
        if ! kill -0 $MOTOR_PID 2>/dev/null; then
            echo "⚠️ Motor driver stopped, restarting..."
            ros2 run ros2_roboclaw_driver motor_driver &
            MOTOR_PID=$!
        fi
        
        # Check if HTTP bridge is still running  
        if ! kill -0 $HTTP_PID 2>/dev/null; then
            echo "⚠️ HTTP bridge stopped, restarting..."
            python3 http_bridge.py &
            HTTP_PID=$!
        fi
        
        sleep 10
    done
else
    echo "❌ No serial devices found. Hardware not ready."
    echo "🔧 Check USB connections and try again."
    exit 1
fi
