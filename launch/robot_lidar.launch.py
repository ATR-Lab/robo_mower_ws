#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RP LiDAR A1M8'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser',
            description='Frame ID for laser scan data'
        ),
        
        # RPLiDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'serial_baudrate': 115200,
                'angle_compensate': True,
                'scan_mode': 'Standard',
                'auto_standby': True
            }],
            output='screen'
        ),
        
        # RoboClaw Motor Driver
        Node(
            package='ros2_roboclaw_driver',
            executable='roboclaw_driver_node',
            name='roboclaw_driver',
            parameters=[{
                'device_name': '/dev/ttyACM2',
                'baud_rate': 38400,
                'm1_p': 7.26239,
                'm1_i': 1.36838,
                'm1_d': 0.0,
                'm2_p': 7.26239,
                'm2_i': 1.36838,
                'm2_d': 0.0,
                'max_linear': 0.2,
                'timeout': 1.0
            }],
            output='screen'
        ),
        
        # Arduino Encoder Bridge
        ExecuteProcess(
            cmd=['python3', '/home/alwinsdon/ros2_ws/arduino_encoder_bridge.py'],
            output='screen'
        )
    ])
