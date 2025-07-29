#!/usr/bin/env python3

"""
Monitor Arduino Encoder Values from ROS2 Topics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import time

class EncoderMonitor(Node):
    def __init__(self):
        super().__init__('encoder_monitor')
        
        # Subscribers
        self.encoder_sub = self.create_subscription(
            Int64, 'arduino_encoder', self.encoder_callback, 10)
        self.delta_sub = self.create_subscription(
            Int64, 'arduino_encoder_delta', self.delta_callback, 10)
        
        self.current_count = 0
        self.current_delta = 0
        self.last_update = time.time()
        
        # Timer to display values
        self.timer = self.create_timer(0.5, self.display_values)
        
        print("=== Arduino Encoder Monitor ===")
        print("Monitoring topics: /arduino_encoder and /arduino_encoder_delta")
        print("Press Ctrl+C to exit\n")

    def encoder_callback(self, msg):
        self.current_count = msg.data
        self.last_update = time.time()

    def delta_callback(self, msg):
        self.current_delta = msg.data

    def display_values(self):
        current_time = time.time()
        age = current_time - self.last_update
        
        status = "ACTIVE" if age < 1.0 else "STALE"
        
        print(f"\rEncoder: {self.current_count:8d} | Delta: {self.current_delta:+4d} | Status: {status}", end="", flush=True)
        
        if self.current_delta != 0:
            print(f" *** MOVEMENT DETECTED ***", end="")

def main(args=None):
    rclpy.init(args=args)
    
    node = EncoderMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down encoder monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
