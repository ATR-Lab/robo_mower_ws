# ROS2 RoboClaw Dual Motor Control with Real-Time Encoder Feedback

� **FULLY OPERATIONAL & TESTED** ✅ - Real-time encoder monitoring during motor movement successfully implemented.

## 🚀 Quick Start

### Hardware Setup
- **RoboClaw 2x60a v4.3.6** → USB (`/dev/ttyACM0`)
- **Arduino Uno** → USB (`/dev/ttyACM1`) 
- **Encoders** → Arduino pins (NOT RoboClaw)
- **Motors** → RoboClaw terminals

### Run the System
```bash
# Terminal 1: RoboClaw driver
cd ~/ros2_ws && source install/setup.bash
ros2 run ros2_roboclaw_driver ros2_roboclaw_driver_node

# Terminal 2: Arduino encoder bridge
cd ~/ros2_ws && python3 arduino_encoder_bridge.py

# Terminal 3: Monitor encoders
ros2 topic echo /arduino_encoder

# Terminal 4: Send commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 📊 Proven Performance
- ✅ **20Hz encoder updates** during motor movement
- ✅ **Real-time position feedback** in radians  
- ✅ **Motor control** via `/cmd_vel` topic
- ✅ **Dual-path architecture** (separate motor/encoder streams)

## ⚡ Core Files

- `FINAL_SYSTEM_DOCUMENTATION.md` - Complete system documentation
- `arduino_encoder_bridge.py` - Python bridge for Arduino encoder data
- `real_time_encoder_viewer.py` - Real-time encoder monitoring
- `arduino_roboclaw_usb_bridge/encoder_publisher.ino` - Arduino encoder sketch
- `src/ros2_roboclaw_driver/` - RoboClaw ROS2 driver package

## 🌐 Remote Control Ready
Instructions included for PC → Raspberry Pi remote operation.

## 📋 Complete Documentation
See [FINAL_SYSTEM_DOCUMENTATION.md](FINAL_SYSTEM_DOCUMENTATION.md) for full setup, wiring diagrams, and troubleshooting.

```
Motor Commands (/cmd_vel) → RoboClaw → Motor
Encoder → Arduino → Serial → Python Bridge → /arduino_encoder topic
```

## Key Features

✅ Real-time encoder feedback at 20Hz  
✅ Independent motor control and encoder streams  
✅ Complete ROS2 integration  
✅ Single motor with encoder support  
✅ Ready-to-use scripts and documentation  

---
*System Status: FULLY OPERATIONAL*
