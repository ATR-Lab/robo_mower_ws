# Remote Lawnmower Control Setup Guide

## Overview
This guide shows how to control your lawnmower robot from another PC over WiFi.

**Robot PC (Current Computer):** 192.168.2.144
**Control PC:** Your other computer

## Option 1: ROS2 Network Control (Advanced)

### On Robot PC (This Computer):
1. **Setup robot for remote access:**
   ```bash
   ./setup_remote_robot.sh
   ```

2. **Start the motor driver:**
   ```bash
   ros2 run ros2_roboclaw_driver motor_driver
   ```

### On Control PC:
1. **Install ROS2** (same version - Jazzy recommended)
2. **Copy these files to control PC:**
   - `setup_control_pc.sh`
   - `remote_control.py`

3. **Setup control PC:**
   ```bash
   ./setup_control_pc.sh
   ```

4. **Control the robot:**
   ```bash
   python3 remote_control.py keyboard
   ```

## Option 2: Standalone HTTP Control (Easy - No ROS2 needed on control PC)

### On Robot PC (This Computer):
1. **Install Flask:**
   ```bash
   pip install flask
   ```

2. **Start the HTTP bridge:**
   ```bash
   python3 http_bridge.py
   ```

3. **Start motor driver** (in another terminal):
   ```bash
   ./setup_remote_robot.sh
   ros2 run ros2_roboclaw_driver motor_driver
   ```

### On Control PC:
1. **Install Python requests:**
   ```bash
   pip install requests
   ```

2. **Copy these files to control PC:**
   - `standalone_control.py`

3. **Control the robot:**
   ```bash
   python3 standalone_control.py standalone
   ```

4. **Or use web browser:**
   - Open: http://192.168.2.144:8080
   - Click buttons to control robot

## Option 3: SSH Control (Quick Test)

### From Control PC:
```bash
ssh username@192.168.2.144
cd ros2_ws
python3 remote_control.py keyboard
```

## Commands:
- **W** - Move Forward
- **S** - Move Backward  
- **A** - Turn Left
- **D** - Turn Right
- **SPACE** - Stop
- **Q** - Quit

## Troubleshooting:

### Robot not responding:
1. Check WiFi connection on both PCs
2. Verify robot IP: `hostname -I`
3. Test ping from control PC: `ping 192.168.2.144`
4. Make sure motor driver is running
5. Check USB connections on robot

### Network issues:
- Both PCs must be on same WiFi network
- Check firewall settings
- Try different ROS_DOMAIN_ID (42)

### Hardware issues:
- Battery connected and charged
- USB devices detected: `lsusb`
- Serial ports created: `ls /dev/ttyACM*`

## File Summary:
- `setup_remote_robot.sh` - Configure robot PC for remote access
- `setup_control_pc.sh` - Configure control PC for ROS2 method
- `remote_control.py` - ROS2 keyboard control script
- `standalone_control.py` - HTTP keyboard control (no ROS2 needed)
- `http_bridge.py` - HTTP to ROS2 bridge (runs on robot)

## Security Note:
This setup is for local network use only. Do not expose to internet without proper security measures.
