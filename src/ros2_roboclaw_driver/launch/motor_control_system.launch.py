#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get package directories
    ros2_roboclaw_driver_dir = get_package_share_directory('ros2_roboclaw_driver')
    
    # Declare launch arguments
    arduino_port_arg = DeclareLaunchArgument(
        'arduino_port',
        default_value='/dev/ttyACM0',
        description='Arduino USB port for RoboClaw bridge'
    )
    
    encoder_port_arg = DeclareLaunchArgument(
        'encoder_port', 
        default_value='/dev/ttyACM1',
        description='Arduino USB port for encoder data (if using second Arduino)'
    )
    
    # RoboClaw Driver Node (via Arduino USB bridge)
    roboclaw_driver_node = Node(
        package='ros2_roboclaw_driver',
        executable='ros2_roboclaw_driver_node',
        name='roboclaw_driver',
        parameters=[os.path.join(ros2_roboclaw_driver_dir, 'config', 'motor_driver.yaml')],
        output='screen',
        remappings=[
            ('cmd_vel', 'motor_cmd_vel'),  # Remap to avoid conflicts
        ]
    )
    
    # Arduino Encoder Bridge Node
    encoder_bridge_node = Node(
        package='arduino_encoder_bridge',
        executable='arduino_encoder_bridge',
        name='arduino_encoder_bridge',
        parameters=[
            {'arduino_port': LaunchConfiguration('encoder_port')},
            {'baud_rate': 115200},
            {'publish_rate': 20}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        arduino_port_arg,
        encoder_port_arg,
        roboclaw_driver_node,
        encoder_bridge_node,
    ]) 