#!/usr/bin/env python3

"""
Real-time Encoder Viewer
Shows encoder values from both sources in a clean format
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import os

class EncoderViewer(Node):
    def __init__(self):
        super().__init__('encoder_viewer')
        
        # Subscribe to Arduino encoder
        self.arduino_sub = self.create_subscription(
            JointState, 'arduino_encoder', self.arduino_callback, 10)
        
        # Subscribe to RoboClaw joint states
        self.roboclaw_sub = self.create_subscription(
            JointState, 'joint_states', self.roboclaw_callback, 10)
        
        self.arduino_position1 = 0.0
        self.arduino_position2 = 0.0
        self.roboclaw_position = 0.0
        self.arduino_count = 0
        self.roboclaw_count = 0
        
        # Timer to update display
        self.timer = self.create_timer(0.1, self.update_display)
        
        print("Real-time Dual Encoder Viewer Started")
        print("=" * 60)
    
    def arduino_callback(self, msg):
        if len(msg.position) >= 2:
            self.arduino_position1 = msg.position[0]  # Right wheel
            self.arduino_position2 = msg.position[1]  # Left wheel
            self.arduino_count += 1
        elif len(msg.position) > 0:
            # Fallback for single encoder
            self.arduino_position1 = msg.position[0]
            self.arduino_count += 1
    
    def roboclaw_callback(self, msg):
        if len(msg.position) > 0:
            # Use the first wheel position
            self.roboclaw_position = msg.position[0]
            self.roboclaw_count += 1
    
    def update_display(self):
        # Clear screen
        os.system('clear')
        
        print("🔄 REAL-TIME DUAL ENCODER VIEWER 🔄")
        print("=" * 70)
        print(f"⏱️  Time: {time.strftime('%H:%M:%S')}")
        print()
        
        print("📡 ARDUINO ENCODERS:")
        print(f"   Right Wheel (M1): {self.arduino_position1:>12.3f} radians")
        print(f"   Right Counts:     {int(self.arduino_position1 * 1000 / (2 * 3.14159)):>12} encoder counts")
        print(f"   Left Wheel (M2):  {self.arduino_position2:>12.3f} radians")
        print(f"   Left Counts:      {int(self.arduino_position2 * 1000 / (2 * 3.14159)):>12} encoder counts")
        print(f"   Updates:          {self.arduino_count:>12} messages")
        print()
        
        print("🤖 ROBOCLAW JOINT STATE:")
        print(f"   Position: {self.roboclaw_position:>12.3f} radians")
        print(f"   Updates:  {self.roboclaw_count:>12} messages") 
        print()
        
        # Calculate differences
        diff1 = abs(self.arduino_position1 - self.roboclaw_position)
        diff2 = abs(self.arduino_position2 - self.roboclaw_position)
        print(f"📊 DIFFERENCES:")
        print(f"   Right wheel: {diff1:>12.3f} radians")
        print(f"   Left wheel:  {diff2:>12.3f} radians")
        print()
        print("Press Ctrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    
    node = EncoderViewer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Encoder viewer stopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
