# ROS2 Workspace Structure

This workspace contains a complete robotics system with camera, LIDAR, motor control, and mapping capabilities.

## Directory Structure

```
ros2_ws/
├── arduino_sketches/          # Arduino code for hardware interfaces
├── build/                     # ROS2 build artifacts
├── camera_system/            # Complete camera publishing/subscribing system
├── documentation/            # All documentation files
├── install/                  # ROS2 installation files
├── launch/                   # ROS2 launch files
├── log/                      # Build and runtime logs
├── rviz_configs/            # RViz visualization configurations
├── scripts/                 # Utility and setup scripts
└── src/                     # ROS2 source packages
```

## Main Components

### Camera System (`camera_system/`)
- `camera_publisher.py` - Main camera publishing node
- `camera_subscriber.py` - Advanced camera subscriber with cv_bridge
- `simple_camera_subscriber.py` - Simple subscriber without dependencies
- `setup_camera.sh` - Camera system setup script
- Complete documentation and success reports

### Robot Control
- `robot_api.py` - Main robot API
- `robot_command_server.py` - Command server
- `pi_motor_server.py` - Motor control server
- `remote_control.py` - Remote control interface
- `standalone_control.py` - Standalone control mode

### Mapping and Navigation
- `slam_params.yaml` - SLAM parameters
- Various launch files for different system configurations
- RViz configurations for visualization

### Utilities (`scripts/`)
- Setup scripts for different components
- Emergency stop and hardware testing
- Real-time monitoring tools

## Quick Start

1. **Camera System**: Use files in `camera_system/` directory
2. **Robot Control**: Start with `robot_api.py`
3. **Mapping**: Use launch files in `launch/` directory
4. **Documentation**: Check `documentation/` for detailed guides

## Cleaned Up Items

- Removed Python `__pycache__` directories
- Removed temporary test images
- Removed duplicate files
- Organized files by functionality
- Cleared old build logs
