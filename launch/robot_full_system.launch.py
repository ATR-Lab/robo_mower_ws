#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # LiDAR node - starts immediately
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        ),
        
        # Motor driver node - starts after short delay
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros2_roboclaw_driver',
                    executable='roboclaw_driver_node',
                    name='roboclaw_driver',
                    parameters=['/home/alwinsdon/ros2_ws/src/ros2_roboclaw_driver/config/motor_driver.yaml'],
                    output='screen'
                )
            ]
        ),
        
        # Static transform from base_link to laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_broadcaster',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),
        
        # Base footprint transform
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        
        # Robot State Publisher (for robot model)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'use_sim_time': False,
                'robot_description': """<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_footprint"/>
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>
  <link name="laser">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="0 0 0.05"/>
  </joint>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0 0 0.1"/>
  </joint>
</robot>"""
            }],
            output='screen'
        ),
        
        # SLAM Toolbox for mapping
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    parameters=[{
                        'use_sim_time': False,
                        'slam_params_file': '/home/alwinsdon/ros2_ws/slam_params.yaml'
                    }],
                    output='screen'
                )
            ]
        ),
        
        # RViz for visualization - starts last
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', '/home/alwinsdon/ros2_ws/robot_config.rviz'],
                    output='screen'
                )
            ]
        ),
    ])
