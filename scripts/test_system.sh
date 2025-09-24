#!/bin/bash

# Test script to verify all system components
echo "🧪 Testing ROS2 RoboClaw System Components..."

# Test 1: Check ROS2 installation
echo "1️⃣ Checking ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "   ✅ ROS2 found: $(ros2 --version)"
else
    echo "   ❌ ROS2 not found"
    exit 1
fi

# Test 2: Check Python dependencies
echo "2️⃣ Checking Python dependencies..."
python3 -c "import serial; print('   ✅ PySerial available')" 2>/dev/null || echo "   ❌ PySerial not found - run: pip3 install pyserial"

# Test 3: Check workspace build
echo "3️⃣ Checking workspace build..."
if [ -d "install" ]; then
    echo "   ✅ Workspace built"
else
    echo "   ❌ Workspace not built - run: colcon build"
fi

# Test 4: Check for hardware
echo "4️⃣ Checking for connected hardware..."
if ls /dev/ttyACM* 1> /dev/null 2>&1; then
    echo "   ✅ USB devices found:"
    for device in /dev/ttyACM*; do
        echo "      - $device"
    done
else
    echo "   ⚠️  No /dev/ttyACM* devices found"
    echo "      Connect RoboClaw (should be /dev/ttyACM0)"
    echo "      Connect Arduino (should be /dev/ttyACM1)"
fi

# Test 5: Check critical files
echo "5️⃣ Checking critical files..."
files=(
    "arduino_encoder_bridge.py"
    "real_time_encoder_viewer.py" 
    "arduino_roboclaw_usb_bridge/encoder_publisher.ino"
    "FINAL_SYSTEM_DOCUMENTATION.md"
)

for file in "${files[@]}"; do
    if [ -f "$file" ]; then
        echo "   ✅ $file"
    else
        echo "   ❌ Missing: $file"
    fi
done

# Test 6: Check ROS2 driver
echo "6️⃣ Checking ROS2 driver package..."
if [ -d "src/ros2_roboclaw_driver" ]; then
    echo "   ✅ RoboClaw driver package found"
else
    echo "   ❌ RoboClaw driver package missing"
fi

echo ""
echo "🏁 Test complete!"
echo "📖 See FINAL_SYSTEM_DOCUMENTATION.md for setup instructions"
