#!/usr/bin/env python3
"""
Remote Motor Control Script
Send motor commands over network to ROS2 system
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import time

class RemoteMotorController(Node):
    def __init__(self):
        super().__init__('remote_motor_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('🎮 Remote Motor Controller Ready!')

    def send_command(self, linear_x=0.0, angular_z=0.0):
        """Send velocity command to motors"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 Sent: Linear={linear_x}, Angular={angular_z}')

def keyboard_control():
    """Interactive keyboard control mode"""
    print("\n🎮 Remote Lawnmower Control")
    print("Setting up ROS2 network...")
    
    # Configure ROS2 for network communication
    import os
    os.environ['ROS_DOMAIN_ID'] = '42'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp' 
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'SUBNET'
    
    try:
        rclpy.init()
        controller = RemoteMotorController()
        print("✅ Connected to robot network!")
        
        # Give time for discovery
        time.sleep(2)
        
    except Exception as e:
        print(f"❌ ROS2 initialization failed: {e}")
        print("⚠️  Make sure robot is running and on same network")
        return
    
    print("Controls:")
    print("  W - Forward")
    print("  S - Backward") 
    print("  A - Turn Left")
    print("  D - Turn Right")
    print("  Q - Quit")
    print("  SPACE - Stop")
    print("\nPress keys and ENTER:")
    
    try:
        while True:
            key = input(">>> ").strip().lower()
            
            if key == 'w':
                print("🔼 Sending Forward Command...")
                controller.send_command(0.3, 0.0)  # Forward
                print("✅ Forward command sent!")
            elif key == 's':
                print("🔽 Sending Backward Command...")
                controller.send_command(-0.3, 0.0)  # Backward
                print("✅ Backward command sent!")
            elif key == 'a':
                print("◀️ Sending Turn Left Command...")
                controller.send_command(0.0, 1.5)  # Increased angular velocity
                print("✅ Turn left command sent!")
            elif key == 'd':
                print("▶️ Sending Turn Right Command...")
                controller.send_command(0.0, -1.5)  # Increased angular velocity
                print("✅ Turn right command sent!")
            elif key == ' ' or key == 'stop':
                print("⏹️ Sending Stop Command...")
                controller.send_command(0.0, 0.0)  # Stop
                print("✅ Stop command sent!")
            elif key == 'q' or key == 'quit':
                print("🛑 Stopping and quitting...")
                controller.send_command(0.0, 0.0)  # Stop before quit
                print("👋 Goodbye!")
                break
            elif key == 'status':
                print("📊 Checking system status...")
                # Add status check here if needed
            else:
                print("❌ Unknown command. Use W/A/S/D/SPACE/Q")
                
    except KeyboardInterrupt:
        print("\n🛑 Stopping motors...")
        controller.send_command(0.0, 0.0)  # Stop on Ctrl+C
        print("\n👋 Goodbye!")
    
    try:
        controller.destroy_node()
        rclpy.shutdown()
    except:
        pass

def main():
    # Check if keyboard mode requested
    if len(sys.argv) == 2 and sys.argv[1] == "keyboard":
        keyboard_control()
        return
    elif len(sys.argv) == 2 and sys.argv[1] == "test":
        # Quick test mode - just send one forward command
        print("🚀 Quick Test Mode - Sending forward command...")
        try:
            rclpy.init()
            controller = RemoteMotorController()
            controller.send_command(0.3, 0.0)  # Forward
            print("✅ Forward command sent!")
            time.sleep(1)
            controller.send_command(0.0, 0.0)  # Stop
            print("⏹️ Stop command sent!")
            controller.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"❌ Test failed: {e}")
        return
        
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python3 remote_control.py keyboard              # Interactive keyboard mode")
        print("  python3 remote_control.py test                  # Quick test mode")
        print("  python3 remote_control.py <linear_x> <angular_z> # Direct command")
        print("\nDirect command examples:")
        print("  python3 remote_control.py 0.3 0.0    # Forward")
        print("  python3 remote_control.py -0.3 0.0   # Backward") 
        print("  python3 remote_control.py 0.0 0.5    # Turn left")
        print("  python3 remote_control.py 0.0 -0.5   # Turn right")
        print("  python3 remote_control.py 0.0 0.0    # Stop")
        sys.exit(1)

    linear_x = float(sys.argv[1])
    angular_z = float(sys.argv[2])

    rclpy.init()
    controller = RemoteMotorController()
    
    # Send command
    controller.send_command(linear_x, angular_z)
    
    # Keep alive briefly to ensure message is sent
    time.sleep(0.5)
    
    controller.destroy_node()
    rclpy.shutdown()
    print(f"✅ Command sent: Linear={linear_x} m/s, Angular={angular_z} rad/s")

if __name__ == '__main__':
    main()
