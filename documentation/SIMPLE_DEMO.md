# Simple Demo Setup

## Robot PC (Current Computer - 192.168.2.144):
```bash
# 1. Setup robot
./setup_remote_robot.sh

# 2. Start motor driver
ros2 run ros2_roboclaw_driver motor_driver
```

## Demo PC (Your Control Computer):
```bash
# 1. Clone this repo
git clone https://github.com/alwinsdon/ros2-roboclaw-encoder-system.git
cd ros2-roboclaw-encoder-system

# 2. Setup environment
./setup_control_pc.sh

# 3. Control robot
python3 remote_control.py keyboard
```

## Controls:
- **W** - Forward
- **S** - Backward  
- **A** - Turn Left
- **D** - Turn Right
- **SPACE** - Stop
- **Q** - Quit

## Requirements:
- Both PCs on same WiFi
- ROS2 Jazzy on both PCs
- ROS_DOMAIN_ID=42

That's it! Simple and direct.
