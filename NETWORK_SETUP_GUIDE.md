# ROS2 Network Setup: Daughter PC (Humble) ↔ Raspberry Pi (Jazzy)

## ✅ **COMPATIBILITY CONFIRMED**
ROS2 Humble and Jazzy **CAN communicate** for teleop operations using `geometry_msgs/Twist` messages.

## 🌐 **Complete Setup Guide**

### **STEP 1: Network Setup on Daughter PC (Current PC) - ROS2 Humble**

```bash
# 1. Source ROS2 Humble
source /opt/ros/humble/setup.bash
source install/setup.bash

# 2. Configure ROS2 Network Settings
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 3. Get your IP address
hostname -I

# 4. Test ROS2 is working
ros2 topic list
```

### **STEP 2: Network Setup on Raspberry Pi - ROS2 Jazzy**

```bash
# 1. Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 2. Configure ROS2 Network Settings (SAME as daughter PC)
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# 3. Get Raspberry Pi IP address
hostname -I

# 4. Test ROS2 is working
ros2 topic list
```

### **STEP 3: Test Network Connection**

**On Raspberry Pi:**
```bash
# Start motor driver
ros2 run ros2_roboclaw_driver ros2_roboclaw_driver_node
```

**On Daughter PC (separate terminal):**
```bash
# Test connection by listing topics
ros2 topic list

# You should see topics from both systems including:
# /cmd_vel (for motor commands)
# /arduino_encoder (for encoder feedback)
```

### **STEP 4: Run Teleop Control**

**Option A: Using the built-in teleop node**
```bash
# On Daughter PC
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Run teleop
ros2 run robo_mower_teleop teleop_tank_robot
```

**Option B: Using manual topic commands**
```bash
# Forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Backward  
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn Left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5}}"

# Turn Right
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### **STEP 5: Monitor Robot Status**

```bash
# Monitor encoder feedback
ros2 topic echo /arduino_encoder

# Monitor motor commands
ros2 topic echo /cmd_vel

# Check all active topics
ros2 topic list

# Check nodes
ros2 node list
```

## 🔧 **Environment Setup Scripts**

### Daughter PC Setup (`setup_daughter_pc.sh`)
```bash
#!/bin/bash
# Source this before every session
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
echo "✅ Daughter PC (Humble) ready for network control!"
```

### Raspberry Pi Setup (`setup_raspberry_pi.sh`)  
```bash
#!/bin/bash
# Source this on Raspberry Pi before every session
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
echo "✅ Raspberry Pi (Jazzy) ready for network control!"
```

## 🚨 **Important Notes**

1. **Same Network**: Ensure both devices are on the same WiFi network
2. **Same Domain ID**: Both devices MUST use `ROS_DOMAIN_ID=42`
3. **Same RMW**: Both devices MUST use `rmw_fastrtps_cpp`
4. **Firewall**: Ensure firewalls allow multicast traffic on ports 7400-7500 (UDP)
5. **Session Specific**: Environment variables are terminal-session specific

## 🔍 **Testing Workflow**

1. **Network Connectivity**: `ping <raspberry_pi_ip>`
2. **ROS2 Discovery**: `ros2 topic list` (should show topics from both systems)
3. **Message Flow**: `ros2 topic echo /cmd_vel` while sending commands
4. **Robot Response**: Physical movement should occur

## ⚠️ **Troubleshooting**

- **No topics visible**: Check `ROS_DOMAIN_ID` matches on both systems
- **Commands not working**: Verify `/cmd_vel` topic exists with `ros2 topic info /cmd_vel`
- **Network issues**: Check firewall settings and multicast support
- **Discovery problems**: Try `ros2 daemon stop && ros2 daemon start`