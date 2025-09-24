# Demo Setup Instructions

## Scenario: Remote Lawnmower Demo
- **Robot PC:** Will run autonomously (this computer - 192.168.2.144)
- **Demo PC:** Your control computer with ROS2 already installed
- **Setup:** Robot PC won't be accessible during demo

## Pre-Demo Setup (Do this now):

### 1. On Robot PC (This Computer):
```bash
# Make scripts executable
chmod +x auto_start_robot.sh
chmod +x setup_remote_robot.sh

# Install required Python packages
pip install flask requests

# Test the auto-start system
./auto_start_robot.sh
```

### 2. Copy Files to Demo PC:
Transfer these files to your demo/control PC:
- `setup_control_pc.sh`
- `remote_control.py`
- `standalone_control.py`

### 3. Set Robot to Auto-Start (Optional):
To make robot start automatically on boot:
```bash
# Add to startup (choose one method):

# Method A: Add to .bashrc
echo "./home/alwinsdon/ros2_ws/auto_start_robot.sh" >> ~/.bashrc

# Method B: Create systemd service
sudo nano /etc/systemd/system/lawnmower.service
```

## During Demo:

### Option 1: ROS2 Control (Recommended)
On your demo PC:
```bash
# Setup environment
./setup_control_pc.sh

# Control robot
python3 remote_control.py keyboard
```

### Option 2: Web Browser Control (Backup)
1. Open browser on demo PC
2. Go to: http://192.168.2.144:8080
3. Use web buttons to control robot

### Option 3: Standalone Control (No ROS2 topics)
On your demo PC:
```bash
pip install requests
python3 standalone_control.py standalone
```

## Demo Commands:
- **W** - Forward
- **S** - Backward  
- **A** - Turn Left
- **D** - Turn Right
- **SPACE** - Stop
- **Q** - Quit

## Troubleshooting During Demo:

### Can't connect to robot:
1. Check robot IP: Should be 192.168.2.144
2. Ping test: `ping 192.168.2.144`
3. Web test: http://192.168.2.144:8080
4. Both PCs must be on same WiFi

### Robot not responding:
1. Try web interface first
2. Check if robot system is running
3. Use standalone control as backup

### Emergency Stop:
- Press SPACE in any control method
- Or use web interface Stop button
- Physical power button on robot

## Pre-Demo Checklist:
- ✅ Robot battery charged
- ✅ Robot connected to WiFi
- ✅ Auto-start script tested
- ✅ Files copied to demo PC
- ✅ Demo PC has ROS2 installed
- ✅ Both PCs on same network
- ✅ Web interface accessible
- ✅ Control commands tested

## Network Requirements:
- Both PCs on same WiFi network
- Robot IP: 192.168.2.144 (verify with `hostname -I`)
- No firewall blocking ports 8080, 11811
- ROS_DOMAIN_ID=42 on both PCs
