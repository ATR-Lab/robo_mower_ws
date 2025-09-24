#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import time

class FakeLidarNode(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz
        self.angle = 0.0
        
    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser'
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 30.0
        
        # Create a simple room-like environment
        ranges = []
        for i in range(360):
            angle = scan.angle_min + i * scan.angle_increment
            # Create walls at different distances
            if -math.pi/4 < angle < math.pi/4:  # Front wall
                distance = 3.0 + 0.5 * math.sin(self.angle)
            elif math.pi/4 < angle < 3*math.pi/4:  # Right wall  
                distance = 2.0 + 0.3 * math.cos(self.angle)
            elif -3*math.pi/4 < angle < -math.pi/4:  # Left wall
                distance = 2.5 + 0.2 * math.sin(self.angle * 2)
            else:  # Back wall
                distance = 4.0
            
            ranges.append(distance)
        
        scan.ranges = ranges
        scan.intensities = [100.0] * len(ranges)
        
        self.publisher.publish(scan)
        self.angle += 0.1

def main():
    rclpy.init()
    node = FakeLidarNode()
    print("Fake LiDAR node started - publishing to /scan topic")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
