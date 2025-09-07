#!/usr/bin/env python3
"""
Tank Robot Teleoperation Script
Controls a two-motor tank-style robot with differential steering
Uses WASD keys for movement control

Controls:
W/S - Forward/Backward
A/D - Turn Left/Right (differential steering)
Q/E - Pivot Left/Right (one track forward, one backward)
X - Stop
ESC/Ctrl+C - Exit

Author: ROS2 Tank Robot Controller
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time

class TankTeleopNode(Node):
    def __init__(self):
        super().__init__('tank_teleop_node')
        
        # Publisher for cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Movement parameters
        self.linear_speed = 0.3      # m/s - forward/backward speed
        self.angular_speed = 1.5     # rad/s - turning speed (increased for more power)
        self.pivot_speed = 0.8       # m/s - pivot speed (for Q/E) (increased for more power)
        
        # Current twist message
        self.twist = Twist()
        
        # Control flags
        self.running = True
        
        # Key bindings
        self.key_bindings = {
            'w': 'forward',
            's': 'backward', 
            'a': 'turn_left',
            'd': 'turn_right',
            'q': 'pivot_left',
            'e': 'pivot_right',
            'x': 'stop',
            ' ': 'stop',  # spacebar
            '\x1b': 'exit',  # ESC key
            '\x03': 'exit',  # Ctrl+C
        }
        
        self.get_logger().info("Tank Robot Teleoperation Started!")
        self.print_instructions()

    def print_instructions(self):
        instructions = """
╔══════════════════════════════════════════════════════════════╗
║                    TANK ROBOT TELEOP CONTROLS               ║
╠══════════════════════════════════════════════════════════════╣
║  W - Move Forward           │  S - Move Backward             ║
║  A - Turn Left (gentle)     │  D - Turn Right (gentle)       ║
║  Q - Pivot Left (sharp)     │  E - Pivot Right (sharp)       ║
║  X or SPACE - Stop          │  ESC or Ctrl+C - Exit          ║
╠══════════════════════════════════════════════════════════════╣
║  Current Speeds:                                             ║
║  • Linear Speed: {:.1f} m/s                                  ║
║  • Angular Speed: {:.1f} rad/s                               ║
║  • Pivot Speed: {:.1f} m/s                                   ║
╚══════════════════════════════════════════════════════════════╝
        """.format(self.linear_speed, self.angular_speed, self.pivot_speed)
        print(instructions)

    def get_key(self):
        """Get a single keypress from stdin"""
        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return ''

    def process_key(self, key):
        """Process the pressed key and update twist message"""
        if key in self.key_bindings:
            action = self.key_bindings[key]
            
            if action == 'forward':
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                self.get_logger().info("Moving Forward")
                
            elif action == 'backward':
                self.twist.linear.x = -self.linear_speed
                self.twist.angular.z = 0.0
                self.get_logger().info("Moving Backward")
                
            elif action == 'turn_left':
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.angular_speed
                self.get_logger().info("Turning Left")
                
            elif action == 'turn_right':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed
                self.get_logger().info("Turning Right")
                
            elif action == 'pivot_left':
                # Tank pivot: one track forward, one backward
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.angular_speed * 2.0  # Maximum sharp turn
                self.get_logger().info("Pivot Left (Maximum Power)")
                
            elif action == 'pivot_right':
                # Tank pivot: one track forward, one backward
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed * 2.0  # Maximum sharp turn
                self.get_logger().info("Pivot Right (Maximum Power)")
                
            elif action == 'stop':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.get_logger().info("STOP")
                
            elif action == 'exit':
                self.get_logger().info("Exiting...")
                self.running = False
                return False
                
            # Publish the twist message
            self.cmd_vel_pub.publish(self.twist)
            return True
        else:
            # Unknown key pressed
            if key and ord(key) >= 32 and ord(key) <= 126:  # Printable characters
                self.get_logger().warn(f"Unknown key: '{key}' - Press W/A/S/D/Q/E/X or ESC")
            return True

    def run_teleop(self):
        """Main teleoperation loop"""
        # Save terminal settings
        old_attr = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to raw mode for immediate key response
            tty.setraw(sys.stdin.fileno())
            
            while self.running and rclpy.ok():
                key = self.get_key()
                if key:
                    if not self.process_key(key):
                        break
                
                # Spin ROS2 to handle callbacks
                rclpy.spin_once(self, timeout_sec=0.01)
                
        except Exception as e:
            try:
                if rclpy.ok():
                    self.get_logger().error(f"Error in teleop loop: {e}")
                else:
                    print(f"Error in teleop loop: {e}")
            except:
                print(f"Error in teleop loop: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
            
            # Send stop command before exit (only if ROS context is valid)
            try:
                if rclpy.ok() and hasattr(self, 'cmd_vel_pub'):
                    stop_twist = Twist()
                    self.cmd_vel_pub.publish(stop_twist)
                    self.get_logger().info("Tank robot stopped and teleop exited.")
                else:
                    print("Tank robot stopped and teleop exited.")
            except:
                print("Tank robot stopped and teleop exited.")


def main():
    """Main function"""
    # Check for domain ID argument
    domain_id = None
    if len(sys.argv) > 1:
        try:
            domain_id = int(sys.argv[1])
            print(f"Using ROS_DOMAIN_ID: {domain_id}")
        except ValueError:
            print("Invalid domain ID. Using default.")
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create and run teleop node
        teleop_node = TankTeleopNode()
        
        print("Starting tank robot teleoperation...")
        print("Make sure your robot driver is running!")
        print("Press any key to start controlling...")
        
        # Run the teleoperation
        teleop_node.run_teleop()
        
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received. Stopping robot...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Cleanup
        try:
            if 'teleop_node' in locals():
                teleop_node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass
        print("Teleoperation ended.")


if __name__ == '__main__':
    main() 